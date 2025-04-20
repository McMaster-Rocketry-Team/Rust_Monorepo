use core::{
    array,
    cmp::{min, Ordering},
};

use embassy_futures::yield_now;
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    pubsub::{PubSubBehavior, PubSubChannel},
};
use heapless::Vec;
use packed_struct::PackedStructSlice;

use crate::{sensor_reading::SensorReading, time::BootTimestamp};

use super::{
    id::CanBusExtendedId,
    messages::CanBusMessageEnum,
    sender::{TailByte, CAN_CRC, MAX_CAN_MESSAGE_SIZE},
    CanBusRX, CanBusRawMessage,
};

pub struct CanReceiver<M: RawMutex, const N: usize, const SUBS: usize, const Q: usize> {
    channel: PubSubChannel<M, SensorReading<BootTimestamp, CanBusMessageEnum>, N, SUBS, 1>,
}

enum StateMachine {
    Empty,
    MultiFrame {
        id: CanBusExtendedId,
        first_frame_timestamp: f64,
        crc: u16,
        data: Vec<u8, MAX_CAN_MESSAGE_SIZE>,
        message_len: usize,
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

    fn tail_byte_i(message_len: usize, received_len: usize) -> usize {
        min(message_len - received_len, 7)
    }

    fn process_frame(
        &mut self,
        frame: &impl CanBusRawMessage,
    ) -> Option<SensorReading<BootTimestamp, CanBusMessageEnum>> {
        let frame_id = CanBusExtendedId::from_raw(frame.id());
        let frame_data = frame.data();
        match self {
            StateMachine::Empty => {
                let message_len = CanBusMessageEnum::get_message_len(frame_id.message_type)?;
                if message_len <= 7 {
                    // the entire message fits in the first frame
                    let tail_byte_i = Self::tail_byte_i(message_len, 0);
                    let tail_byte = TailByte::unpack_from_slice(&[frame_data[tail_byte_i]]).ok()?;

                    if !(tail_byte.start_of_transfer
                        && tail_byte.end_of_transfer
                        && !tail_byte.toggle)
                    {
                        // Invalid tail byte
                        return None;
                    }

                    let data = &frame_data[..tail_byte_i];
                    return CanBusMessageEnum::deserialize(frame_id.message_type, data)
                        .map(|message| SensorReading::new(frame.timestamp(), message));
                } else {
                    // expect the first frame of a multi-frame message
                    let tail_byte = TailByte::unpack_from_slice(&[frame_data[7]]).ok()?;

                    if !(tail_byte.start_of_transfer
                        && !tail_byte.end_of_transfer
                        && !tail_byte.toggle)
                    {
                        // Invalid tail byte
                        return None;
                    }

                    *self = StateMachine::MultiFrame {
                        id: frame_id,
                        first_frame_timestamp: frame.timestamp(),
                        crc: u16::from_le_bytes([frame_data[0], frame_data[1]]),
                        data: Vec::new(),
                        message_len,
                    };
                    None
                }
            }
            StateMachine::MultiFrame {
                id,
                first_frame_timestamp,
                crc,
                data,
                message_len,
            } => {
                if *id != frame_id {
                    // reset state machine
                    *self = StateMachine::Empty;
                    return self.process_frame(frame);
                }

                let tail_byte_i = Self::tail_byte_i(*message_len, data.len());
                let tail_byte = TailByte::unpack_from_slice(&[frame_data[tail_byte_i]]).ok()?;

                let is_last_frame = *message_len - data.len() <= 7;
                let expected_toggle_bit = ((data.len() - 5) / 7) % 2 == 0;

                if tail_byte.toggle != expected_toggle_bit {
                    // suspect duplicate frame, ignore
                    return None;
                }

                if tail_byte.start_of_transfer || tail_byte.end_of_transfer != is_last_frame {
                    // invalid tail byte
                    *self = StateMachine::Empty;
                    return None;
                }

                data.extend_from_slice(&frame_data[..tail_byte_i]).unwrap();
                if is_last_frame {
                    // last frame, process the message
                    let calculated_crc = CAN_CRC.checksum(&data);
                    if calculated_crc != *crc {
                        // invalid CRC
                        *self = StateMachine::Empty;
                        return None;
                    }

                    let message = CanBusMessageEnum::deserialize(id.message_type, &data)
                        .map(|message| SensorReading::new(*first_frame_timestamp, message));
                    *self = StateMachine::Empty;
                    return message;
                } else {
                    // not the last frame, continue receiving
                    return None;
                }
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
}
