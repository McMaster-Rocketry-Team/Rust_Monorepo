use crc::Crc;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::RawMutex, channel::Channel};
use heapless::Vec;

use crate::can_bus::messages::CanBusMessageEnum;

use super::{id::CanBusExtendedId, CanBusTX};
use packed_struct::prelude::*;

pub const MAX_CAN_MESSAGE_SIZE: usize = 64;

#[derive(PackedStruct)]
#[packed_struct]
pub(super) struct TailByte {
    pub(super) start_of_transfer: bool,
    pub(super) end_of_transfer: bool,
    pub(super) toggle: bool,
    pub(super) transfer_id: ReservedZero<packed_bits::Bits<5>>,
}

impl TailByte {
    pub fn new(start_of_transfer: bool, end_of_transfer: bool, toggle: bool) -> Self {
        Self {
            start_of_transfer,
            end_of_transfer,
            toggle,
            transfer_id: Default::default(),
        }
    }
}

impl Into<u8> for TailByte {
    fn into(self) -> u8 {
        self.pack().unwrap()[0]
    }
}

pub struct CanSender<M: RawMutex, const N: usize> {
    channel: Channel<M, (CanBusExtendedId, Vec<u8, 8>), N>,
    node_type: u8,
    node_id: u16,
}

pub(super) const CAN_CRC: Crc<u16> = Crc::<u16>::new(&crc::CRC_16_IBM_3740);

pub struct CanBusMultiFrameEncoder {
    deserialized_message: [u8; MAX_CAN_MESSAGE_SIZE],
    offset: usize,
    message_len: usize,
    toggle: bool,
    pub crc: u16,
}

impl CanBusMultiFrameEncoder {
    pub fn new(message: CanBusMessageEnum) -> Self {
        let mut deserialized_message = [0u8; MAX_CAN_MESSAGE_SIZE];
        let len = message.serialize(&mut deserialized_message);

        Self {
            crc: CAN_CRC.checksum(&deserialized_message[..len]),
            deserialized_message,
            offset: 0,
            message_len: len,
            toggle: false,
        }
    }
}

impl Iterator for CanBusMultiFrameEncoder {
    type Item = Vec<u8, 8>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.message_len {
            return None;
        }

        let mut data = Vec::new();
        if self.offset == 0 && self.message_len <= 7 {
            // Single frame message
            data.extend_from_slice(&self.deserialized_message[..self.message_len])
                .unwrap();
            data.push(TailByte::new(true, true, false).into()).unwrap();
            self.offset += self.message_len;
        } else {
            // Multi-frame message
            if self.offset == 0 {
                // First frame
                data.extend_from_slice(&self.crc.to_le_bytes()).unwrap();
                data.extend_from_slice(&self.deserialized_message[..5])
                    .unwrap();
                data.push(TailByte::new(true, false, self.toggle).into())
                    .unwrap();
                self.offset += 5;
            } else if self.offset + 7 >= self.message_len {
                // Last frame
                data.extend_from_slice(&self.deserialized_message[self.offset..self.message_len])
                    .unwrap();
                data.push(TailByte::new(false, true, self.toggle).into())
                    .unwrap();
                self.offset = self.message_len;
            } else {
                // Middle frame
                data.extend_from_slice(&self.deserialized_message[self.offset..self.offset + 7])
                    .unwrap();
                data.push(TailByte::new(false, false, self.toggle).into())
                    .unwrap();
                self.offset += 7;
            }

            self.toggle = !self.toggle;
        }

        Some(data)
    }
}

impl<M: RawMutex, const N: usize> CanSender<M, N> {
    pub fn new(node_type: u8, node_id: u16) -> Self {
        Self {
            channel: Channel::new(),
            node_type,
            node_id,
        }
    }

    pub async fn run_daemon(&self, tx: &mut impl CanBusTX, node_type: u8, node_id: u16) {
        loop {
            let (mut id, data) = self.channel.receive().await;
            id.node_type = node_type.into();
            id.node_id = node_id.into();
            let result = tx.send(id.into(), &data).await;
            if let Err(e) = result {
                log_error!("Failed to send CAN frame: {:?}", e);
                yield_now().await;
            }
        }
    }

    pub async fn send(&self, message: CanBusMessageEnum) -> u16 {
        let id = message.get_id(self.node_type, self.node_id);

        let multi_frame_encoder = CanBusMultiFrameEncoder::new(message);
        let crc = multi_frame_encoder.crc;
        for data in multi_frame_encoder {
            self.channel.send((id, data)).await;
        }
        crc
    }
}
