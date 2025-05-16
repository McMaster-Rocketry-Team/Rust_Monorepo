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
    CanBusFrame, CanBusRX,
    id::CanBusExtendedId,
    messages::CanBusMessageEnum,
    sender::{CAN_CRC, MAX_CAN_MESSAGE_SIZE, TailByte},
};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReceivedCanBusMessage {
    pub id: CanBusExtendedId,
    pub crc: u16,
    pub message: CanBusMessageEnum,
}

enum StateMachine {
    Empty,
    MultiFrame {
        id: CanBusExtendedId,
        first_frame_timestamp_us: u64,
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
        frame: &impl CanBusFrame,
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
                    frame.timestamp_us(),
                    ReceivedCanBusMessage {
                        id: frame_id,
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

                let mut data = Vec::new();
                data.extend_from_slice(&frame_data[2..frame_data.len() - 1])
                    .unwrap();
                *self = StateMachine::MultiFrame {
                    id: frame_id,
                    first_frame_timestamp_us: frame.timestamp_us(),
                    crc: u16::from_le_bytes([frame_data[0], frame_data[1]]),
                    data,
                };
                None
            }
            StateMachine::MultiFrame {
                id,
                first_frame_timestamp_us,
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

                let result = data.extend_from_slice(&frame_data[..frame_data.len() - 1]);
                if result.is_err() {
                    // buffer overflow
                    *self = StateMachine::Empty;
                    return None;
                }
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
                                *first_frame_timestamp_us,
                                ReceivedCanBusMessage {
                                    id: frame_id,
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

pub struct CanBusMultiFrameDecoder<const Q: usize> {
    state_machines: [StateMachine; Q],
    accepted_message_types: Option<Vec<u8, 32>>
}

impl<const Q: usize> CanBusMultiFrameDecoder<Q> {
    /// if accepted_message_types is None, accept all messages
    pub fn new(accepted_message_types:  Option<Vec<u8, 32>>) -> Self {
        Self {
            state_machines: array::from_fn(|_| StateMachine::new()),
            accepted_message_types
        }
    }

    pub fn process_frame(
        &mut self,
        frame: &impl CanBusFrame,
    ) -> Option<SensorReading<BootTimestamp, ReceivedCanBusMessage>> {
        let id = CanBusExtendedId::from_raw(frame.id());
        if let Some(accepted_message_types) = &self.accepted_message_types{
            if !accepted_message_types.contains(&id.message_type) {
                return None;
            }
        }

        for state_machine in &mut self.state_machines {
            if state_machine.has_same_id(id) {
                if let Some(reading) = state_machine.process_frame(frame) {
                    return Some(reading);
                }
                return None;
            }
        }

        log_warn!("No empty state machine left, discarding least recent used state machine");
        let lru_state_machine = self
            .state_machines
            .iter_mut()
            .min_by(|a, b| match (a, b) {
                (StateMachine::Empty, StateMachine::Empty) => Ordering::Equal,
                (StateMachine::Empty, StateMachine::MultiFrame { .. }) => Ordering::Less,
                (StateMachine::MultiFrame { .. }, StateMachine::Empty) => Ordering::Greater,
                (
                    StateMachine::MultiFrame {
                        first_frame_timestamp_us: ts1,
                        ..
                    },
                    StateMachine::MultiFrame {
                        first_frame_timestamp_us: ts2,
                        ..
                    },
                ) => ts1.partial_cmp(ts2).unwrap(),
            })
            .unwrap();

        lru_state_machine.process_frame(frame)
    }
}

pub struct CanReceiver<M: RawMutex, const N: usize, const SUBS: usize> {
    channel: PubSubChannel<M, SensorReading<BootTimestamp, ReceivedCanBusMessage>, N, SUBS, 1>,
}

impl<M: RawMutex, const N: usize, const SUBS: usize> CanReceiver<M, N, SUBS> {
    pub fn new() -> Self {
        Self {
            channel: PubSubChannel::new(),
        }
    }

    pub async fn run_daemon<R: CanBusRX, const Q: usize>(&self, rx: &mut R, accepted_message_types:  Option<Vec<u8, 32>>) {
        let mut decoder = CanBusMultiFrameDecoder::<Q>::new(accepted_message_types);

        loop {
            match rx.receive().await {
                Ok(frame) => {
                    if let Some(message) = decoder.process_frame(&frame) {
                        self.channel.publish_immediate(message);
                    }
                }
                Err(e) => {
                    log_error!("Failed to receive CAN frame: {:?}", e);
                    yield_now().await;
                }
            }
        }
    }

    pub fn subscriber(
        &self,
    ) -> Option<Subscriber<M, SensorReading<BootTimestamp, ReceivedCanBusMessage>, N, SUBS, 1>>
    {
        self.channel.subscriber().ok()
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        can_bus::{
            messages::{amp_status::PowerOutputStatus, payload_eps_status::*},
            sender::CanBusMultiFrameEncoder,
        },
        tests::init_logger,
    };

    use super::*;

    #[test]
    fn multi_frame_encode_and_decode() {
        init_logger();

        let message = CanBusMessageEnum::PayloadEPSStatus(PayloadEPSStatusMessage::new(
            1,
            2.0,
            3,
            4.0,
            PayloadEPSOutputStatus {
                current_ma: 5,
                overwrote: false,
                status: PowerOutputStatus::Disabled,
            },
            PayloadEPSOutputStatus {
                current_ma: 6,
                overwrote: false,
                status: PowerOutputStatus::PowerGood,
            },
            PayloadEPSOutputStatus {
                current_ma: 7,
                overwrote: false,
                status: PowerOutputStatus::PowerBad,
            },
        ));

        let id = message.get_id(0, 1);
        let id: u32 = id.into();
        let encoder = CanBusMultiFrameEncoder::new(message);
        let encoder_crc = encoder.crc;

        let mut decoder = CanBusMultiFrameDecoder::<1>::new(None);
        let mut decoded_message: Option<SensorReading<BootTimestamp, ReceivedCanBusMessage>> = None;
        for data in encoder {
            let frame = (0u64, id, data.as_slice());
            decoded_message = decoder.process_frame(&frame);
        }

        let decoded_message = decoded_message.unwrap();
        assert_eq!(decoded_message.data.crc, encoder_crc);
        log_info!("Decoded message: {:?}", decoded_message);
    }
}
