use crc::Crc;
use embassy_futures::{
    select::{Either, select},
    yield_now,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    channel::Channel,
    pipe::Pipe,
};
use heapless::Vec;

use crate::can_bus::messages::{CanBusMessageEnum, LOG_MESSAGE_TYPE};

use super::{CanBusTX, id::CanBusExtendedId};
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

pub(super) const CAN_CRC: Crc<u16> = Crc::<u16>::new(&crc::CRC_16_IBM_3740);

pub struct CanBusMultiFrameEncoder {
    serialized_message: [u8; MAX_CAN_MESSAGE_SIZE],
    offset: usize,
    message_len: usize,
    toggle: bool,
    pub crc: u16,
}

impl CanBusMultiFrameEncoder {
    pub fn new(message: CanBusMessageEnum) -> Self {
        let mut serialized_message = [0u8; MAX_CAN_MESSAGE_SIZE];
        let len = message.serialize(&mut serialized_message);

        Self {
            crc: CAN_CRC.checksum(&serialized_message[..len]),
            serialized_message,
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
            data.extend_from_slice(&self.serialized_message[..self.message_len])
                .unwrap();
            data.push(TailByte::new(true, true, false).into()).unwrap();
            self.offset += self.message_len;
        } else {
            // Multi-frame message
            if self.offset == 0 {
                // First frame
                data.extend_from_slice(&self.crc.to_le_bytes()).unwrap();
                data.extend_from_slice(&self.serialized_message[..5])
                    .unwrap();
                data.push(TailByte::new(true, false, self.toggle).into())
                    .unwrap();
                self.offset += 5;
            } else if self.offset + 7 >= self.message_len {
                // Last frame
                data.extend_from_slice(&self.serialized_message[self.offset..self.message_len])
                    .unwrap();
                data.push(TailByte::new(false, true, self.toggle).into())
                    .unwrap();
                self.offset = self.message_len;
            } else {
                // Middle frame
                data.extend_from_slice(&self.serialized_message[self.offset..self.offset + 7])
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

pub struct CanSender<M: RawMutex, const N: usize = 10, const PN: usize = 1024> {
    channel: Channel<M, (CanBusExtendedId, Vec<u8, 8>), N>,
    node_type: u8,
    node_id: u16,
    log_frame_id: u32,
    log_pipe: Option<&'static Pipe<CriticalSectionRawMutex, PN>>,
}

impl<M: RawMutex, const N: usize, const PN: usize> CanSender<M, N, PN> {
    pub fn new(
        node_type: u8,
        node_id: u16,
        log_pipe: Option<&'static Pipe<CriticalSectionRawMutex, PN>>,
    ) -> Self {
        Self {
            channel: Channel::new(),
            node_type,
            node_id,
            log_frame_id: CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, node_type, node_id).into(),
            log_pipe,
        }
    }

    pub async fn run_daemon(&self, tx: &mut impl CanBusTX) {
        let mut send_tx_frame = async |id: u32, data: &[u8]| {
            let result = tx.send(id, &data).await;

            if let Err(e) = result {
                log_error!("Failed to send CAN frame: {:?}", e);
                yield_now().await;
            }
        };

        if let Some(log_pipe) = self.log_pipe {
            let mut buffer = [0u8; 8];
            loop {
                match select(self.channel.receive(), log_pipe.read(&mut buffer)).await {
                    Either::First((id, data)) => send_tx_frame(id.into(), &data).await,
                    Either::Second(len) => send_tx_frame(self.log_frame_id, &buffer[..len]).await,
                }
            }
        } else {
            loop {
                let (id, data) = self.channel.receive().await;
                send_tx_frame(id.into(), &data).await;
            }
        }
    }

    pub fn send(&self, message: CanBusMessageEnum) -> u16 {
        let id = message.get_id(self.node_type, self.node_id);

        let multi_frame_encoder = CanBusMultiFrameEncoder::new(message);
        let crc = multi_frame_encoder.crc;
        for data in multi_frame_encoder {
            let success = self.channel.try_send((id, data)).is_ok();
            if !success {
                log_warn!("can bus sender buffer overflow");
                break;
            }
        }
        crc
    }
}

#[cfg(not(feature = "bootloader"))]
pub fn create_unix_time_frame_data(timestamp_us: u64) -> [u8; 8] {
    let message: CanBusMessageEnum =
        super::messages::unix_time::UnixTimeMessage { timestamp_us }.into();
    let mut data = [0u8; 8];
    message.serialize(&mut data);
    data[7] = TailByte::new(true, true, false).into();

    data
}
