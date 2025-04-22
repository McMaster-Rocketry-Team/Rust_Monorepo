use core::{array, cmp::Ordering};

use embassy_futures::yield_now;
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    pubsub::{PubSubBehavior, PubSubChannel, Subscriber},
};
use heapless::Vec;
use packed_struct::PackedStructSlice;
use serde::{Deserialize, Serialize};

use crate::{sensor_reading::SensorReading, time::BootTimestamp};

use super::{
    id::CanBusExtendedId,
    messages::CanBusMessageEnum,
    sender::{TailByte, CAN_CRC, MAX_CAN_MESSAGE_SIZE},
    CanBusRX, CanBusRawMessage,
};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReceivedCanBusMessage {
    crc: u16,
    message: CanBusMessageEnum,
}

pub struct CanReceiver<M: RawMutex, const N: usize, const SUBS: usize, const Q: usize> {
    channel: PubSubChannel<M, SensorReading<BootTimestamp, ReceivedCanBusMessage>, N, SUBS, 1>,
}

enum StateMachine {
    Empty,
    MultiFrame {
        id: CanBusExtendedId,
        first_frame_timestamp: f64,
        crc: u16,
        data: Vec<u8, MAX_CAN_MESSAGE_SIZE>,
    },
}

impl StateMachine {
    fn new() -> Self {
        Self::Empty
    }

    fn has_same_id(&self, id: CanBusExtendedId) -> bool {
        match self {
            Self::Empty => false,
            Self::MultiFrame { id: cache_id, .. } => *cache_id == id,
        }
    }

    fn process_frame(
        &mut self,
        frame: &impl CanBusRawMessage,
    ) -> Option<SensorReading<BootTimestamp, ReceivedCanBusMessage>> {
        let frame_id = CanBusExtendedId::from_raw(frame.id());
        let frame_data = frame.data();
        if frame_data.len() == 0 {
            // empty frame, ignore
            return None;
        }

        let tail_byte = TailByte::unpack_from_slice(&[frame_data[frame_data.len() - 1]]).ok()?;
        if tail_byte.start_of_transfer && tail_byte.end_of_transfer {
            // Single frame message
            if tail_byte.toggle {
                // Invalid tail byte
                return None;
            }

            let data = &frame_data[..frame_data.len() - 1];
            return CanBusMessageEnum::deserialize(frame_id.message_type, data).map(|message| {
                SensorReading::new(
                    frame.timestamp(),
                    ReceivedCanBusMessage {
                        crc: CAN_CRC.checksum(data),
                        message,
                    },
                )
            });
        }

        match self {
            StateMachine::Empty => {
                // expect the first frame of a multi-frame message
                if !(tail_byte.start_of_transfer && !tail_byte.end_of_transfer && !tail_byte.toggle)
                {
                    // Invalid tail byte
                    return None;
                }

                *self = StateMachine::MultiFrame {
                    id: frame_id,
                    first_frame_timestamp: frame.timestamp(),
                    crc: u16::from_le_bytes([frame_data[0], frame_data[1]]),
                    data: Vec::new(),
                };
                None
            }
            StateMachine::MultiFrame {
                id,
                first_frame_timestamp,
                crc,
                data,
            } => {
                if *id != frame_id {
                    // reset state machine
                    *self = StateMachine::Empty;
                    return self.process_frame(frame);
                }

                let expected_toggle_bit = ((data.len() - 5) / 7) % 2 == 0;

                if tail_byte.toggle != expected_toggle_bit {
                    // suspect duplicate frame, ignore
                    return None;
                }

                if tail_byte.start_of_transfer {
                    // invalid tail byte
                    return None;
                }

                data.extend_from_slice(&frame_data[..frame_data.len() - 1])
                    .unwrap();
                if tail_byte.end_of_transfer {
                    // last frame, parse the message
                    let calculated_crc = CAN_CRC.checksum(&data);
                    if calculated_crc != *crc {
                        // invalid CRC
                        *self = StateMachine::Empty;
                        return None;
                    }

                    let message =
                        CanBusMessageEnum::deserialize(id.message_type, &data).map(|message| {
                            SensorReading::new(
                                *first_frame_timestamp,
                                ReceivedCanBusMessage {
                                    crc: calculated_crc,
                                    message,
                                },
                            )
                        });
                    *self = StateMachine::Empty;
                    return message;
                }

                None
            }
        }
    }
}

impl<M: RawMutex, const N: usize, const SUBS: usize, const Q: usize> CanReceiver<M, N, SUBS, Q> {
    pub fn new() -> Self {
        Self {
            channel: PubSubChannel::new(),
        }
    }

    pub async fn run_daemon<R: CanBusRX>(&self, rx: &mut R) {
        let mut state_machines: [StateMachine; Q] = array::from_fn(|_| StateMachine::new());
        'outer: loop {
            match rx.receive().await {
                Ok(message) => {
                    let id = CanBusExtendedId::from_raw(message.id());
                    for state_machine in &mut state_machines {
                        if state_machine.has_same_id(id) {
                            if let Some(reading) = state_machine.process_frame(&message) {
                                self.channel.publish_immediate(reading);
                            }
                            continue 'outer;
                        }
                    }

                    let lru_state_machine = state_machines
                        .iter_mut()
                        .min_by(|a, b| match (a, b) {
                            (StateMachine::Empty, StateMachine::Empty) => Ordering::Equal,
                            (StateMachine::Empty, StateMachine::MultiFrame { .. }) => {
                                Ordering::Less
                            }
                            (StateMachine::MultiFrame { .. }, StateMachine::Empty) => {
                                Ordering::Greater
                            }
                            (
                                StateMachine::MultiFrame {
                                    first_frame_timestamp: ts1,
                                    ..
                                },
                                StateMachine::MultiFrame {
                                    first_frame_timestamp: ts2,
                                    ..
                                },
                            ) => ts1.partial_cmp(ts2).unwrap(),
                        })
                        .unwrap();
                    if let Some(reading) = lru_state_machine.process_frame(&message) {
                        self.channel.publish_immediate(reading);
                    }
                }
                Err(e) => {
                    log_error!("Failed to receive CAN frame: {:?}", e);
                    yield_now().await;
                }
            }
        }
    }

    pub async fn subscriber(
        &self,
    ) -> Option<Subscriber<M, SensorReading<BootTimestamp, ReceivedCanBusMessage>, N, SUBS, 1>>
    {
        self.channel.subscriber().ok()
    }
}
