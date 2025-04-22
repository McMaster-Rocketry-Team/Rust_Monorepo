use crc::Crc;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::RawMutex, channel::Channel};
use heapless::Vec;

use crate::can_bus::messages::CanBusMessageEnum;

use super::{id::CanBusExtendedId, messages::CanBusMessage, CanBusTX};
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

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
struct RawCanMessage {
    id: CanBusExtendedId,
    data: Vec<u8, 8>,
}

impl RawCanMessage {
    fn new(id: CanBusExtendedId) -> Self {
        Self {
            id,
            data: Vec::new(),
        }
    }
}

pub struct CanSender<M: RawMutex, const N: usize> {
    channel: Channel<M, RawCanMessage, N>,
}

pub(super) const CAN_CRC: Crc<u16> = Crc::<u16>::new(&crc::CRC_16_IBM_3740);

impl<M: RawMutex, const N: usize> CanSender<M, N> {
    pub fn new() -> Self {
        Self {
            channel: Channel::new(),
        }
    }

    pub async fn run_daemon(&self, tx: &mut impl CanBusTX, node_type: u8, node_id: u16) {
        loop {
            let mut message = self.channel.receive().await;
            message.id.node_type = node_type.into();
            message.id.node_id = node_id.into();
            let result = tx.send(message.id.into(), &message.data).await;
            if let Err(e) = result {
                log_error!("Failed to send CAN frame: {:?}", e);
                yield_now().await;
            }
        }
    }

    pub async fn send<T: CanBusMessage>(&self, message: T) {
        let mut buffer = [0u8; MAX_CAN_MESSAGE_SIZE];
        assert!(T::len() <= buffer.len());

        let id = CanBusExtendedId::new(
            message.priority(),
            CanBusMessageEnum::get_message_type::<T>().unwrap(),
            0,
            0,
        );
        message.serialize(&mut buffer);
        let mut buffer = &buffer[..T::len()];

        if buffer.len() <= 7 {
            let mut message = RawCanMessage::new(id);
            message.data.extend_from_slice(buffer).unwrap();
            message
                .data
                .push(TailByte::new(true, true, false).into())
                .unwrap();
            self.channel.send(message).await;
        } else {
            let mut i = 0u32;
            while buffer.len() > 0 {
                let mut message = RawCanMessage::new(id);
                if i == 0 {
                    // first frame
                    let crc = CAN_CRC.checksum(buffer);
                    message.data.extend_from_slice(&crc.to_le_bytes()).unwrap();
                    message.data.extend_from_slice(&buffer[..5]).unwrap();
                    message
                        .data
                        .push(TailByte::new(true, false, i % 2 == 0).into())
                        .unwrap();

                    buffer = &buffer[5..];
                } else if buffer.len() <= 7 {
                    // last frame
                    message.data.extend_from_slice(buffer).unwrap();
                    message
                        .data
                        .push(TailByte::new(false, true, i % 2 == 0).into())
                        .unwrap();

                    buffer = &[]
                } else {
                    // middle frame
                    message.data.extend_from_slice(buffer).unwrap();
                    message
                        .data
                        .push(TailByte::new(false, false, i % 2 == 0).into())
                        .unwrap();

                    buffer = &buffer[7..];
                }

                self.channel.send(message).await;
                i += 1;
            }
        }
    }
}
