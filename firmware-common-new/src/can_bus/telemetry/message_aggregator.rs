use core::convert::Infallible;

use crate::{
    can_bus::{
        id::CanBusExtendedId,
        messages::{CanBusMessageEnum, DATA_TRANSFER_MESSAGE_TYPE},
        sender::MAX_CAN_MESSAGE_SIZE,
    },
    heatshrink::{HeatshrinkError, HeatshrinkWrapper},
};
use heapless::Vec;
use heatshrink::{decoder::HeatshrinkDecoder, encoder::HeatshrinkEncoder};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", bit_numbering = "msb0", size_bytes = "3")]
struct ChunkHeader {
    // 01 indicate this chunk is a message aggregator chunk
    #[packed_field(element_size_bits = "2")]
    _reserved: u8,
    overrun: bool,
    #[packed_field(element_size_bits = "10")]
    compressed_len: u16,
    #[packed_field(element_size_bits = "10")]
    uncompressed_len: u16,
}

impl ChunkHeader {
    fn new(overrun: bool, compressed_len: usize, uncompressed_len: usize) -> Self {
        Self {
            _reserved: 0b01,
            overrun,
            compressed_len: compressed_len as u16,
            uncompressed_len: uncompressed_len as u16,
        }
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..3]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq, Eq)]
struct MessageKey {
    node_type: u8,
    node_id: u16,
}

impl MessageKey {
    pub fn from_id(id: &CanBusExtendedId) -> Self {
        Self {
            node_type: id.node_type,
            node_id: id.node_id,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
struct MessageEntry {
    key: MessageKey,
    message: CanBusMessageEnum,
    count: usize,
    last_received_time: u64,
}

impl MessageEntry {
    fn encoded_len(&self) -> usize {
        return 3 + CanBusMessageEnum::serialized_len(self.message.get_message_type()).unwrap();
    }

    fn encode<E, F: FnMut(&[u8]) -> Result<(), E>>(&self, mut sink: F) -> Result<(), E> {
        let header = MessageHeader {
            message_type: self.message.get_message_type(),
            node_type: self.key.node_type,
            node_id: self.key.node_id,
            count: self.count.min(255) as u8,
        };
        let mut buffer = [0u8; MAX_CAN_MESSAGE_SIZE];
        header.serialize(&mut buffer);
        sink(&buffer[..5])?;

        let len = self.message.serialize(&mut buffer);
        sink(&buffer[..len])?;

        Ok(())
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", bit_numbering = "msb0", size_bytes = "5")]
struct MessageHeader {
    pub message_type: u8,
    pub node_type: u8,
    pub node_id: u16,
    pub count: u8,
}

impl MessageHeader {
    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..5]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}

const AGGREGATOR_LIST_SIZE: usize = 24;

pub struct CanBusMessageAggregator {
    entries: Vec<MessageEntry, AGGREGATOR_LIST_SIZE>,
}

impl CanBusMessageAggregator {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub fn process_message(
        &mut self,
        id: &CanBusExtendedId,
        message: &CanBusMessageEnum,
        received_time: u64,
    ) {
        if id.message_type == DATA_TRANSFER_MESSAGE_TYPE {
            return;
        }
        let key = MessageKey::from_id(id);

        let mut found = false;
        for entry in &mut self.entries {
            if entry.key == key {
                entry.message = message.clone();
                entry.count += 1;
                entry.last_received_time = received_time;

                found = true;
                break;
            }
        }

        if !found {
            let entry = MessageEntry {
                key,
                message: message.clone(),
                count: 1,
                last_received_time: received_time,
            };
            if self.entries.is_full() {
                let least_recent_message = self
                    .entries
                    .iter_mut()
                    .min_by_key(|entry| entry.last_received_time)
                    .unwrap();
                *least_recent_message = entry;
            } else {
                self.entries.push(entry).unwrap();
            }
        }
    }

    // compress at most chunk.len() length of data
    // returns (messages_compressed, compressed_len)
    // errors if compressed_len > chunk.len()
    fn compress_at_most_chunk_len(
        &self,
        chunk: &mut [u8],
    ) -> Result<(usize, usize), HeatshrinkError> {
        let mut hs_enc: HeatshrinkWrapper<'_, HeatshrinkEncoder> = HeatshrinkWrapper::new(chunk);
        let mut entry_iter = self.entries.iter();

        let mut i = 0;
        while let Some(entry) = entry_iter.next() {
            if hs_enc.in_len() + entry.encoded_len() <= hs_enc.out_buffer_len() {
                entry.encode(|data| hs_enc.sink(data))?;
                i += 1;
            } else {
                break;
            }
        }

        let compressed_len = hs_enc.finish()?;
        Ok((i, compressed_len))
    }

    pub fn create_chunk(&mut self, chunk: &mut [u8]) -> usize {
        if chunk.len() <= 3 || self.entries.is_empty() {
            return 0;
        }
        let (header_buffer, chunk) = chunk.split_at_mut(3);

        let (messages_compressed, compressed_len) =
            self.compress_at_most_chunk_len(chunk).unwrap_or((0, 0));

        // fill the free space left with uncompressed data
        let chunk = &mut chunk[compressed_len..];
        let mut offset = 0;
        let mut message_included = messages_compressed;
        let mut entry_iter = self.entries.iter().skip(messages_compressed);
        while let Some(entry) = entry_iter.next() {
            if offset + entry.encoded_len() <= chunk.len() {
                entry
                    .encode(|data| {
                        chunk[offset..(offset + data.len())].copy_from_slice(data);
                        offset += data.len();
                        Ok::<(), Infallible>(())
                    })
                    .unwrap();
                message_included += 1;
            } else {
                break;
            }
        }

        let header = ChunkHeader::new(
            message_included != self.entries.len(),
            compressed_len,
            offset,
        );
        header.serialize(header_buffer);
        self.entries.clear();

        3 + compressed_len + offset
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecodedMessage {
    pub node_type: u8,
    pub node_id: u16,
    pub message: CanBusMessageEnum,
    pub count: usize,
}

impl DecodedMessage {
    /// returns how much should be consumed from chunk, and itself
    fn decode(chunk: &[u8]) -> Result<(usize, Self), DecodeAgregatedMessageError> {
        if chunk.len() < 5 {
            return Err(DecodeAgregatedMessageError::ExpectMore(5 - chunk.len()));
        }

        let header = MessageHeader::deserialize(&chunk[..5])
            .ok_or(DecodeAgregatedMessageError::InvalidMessageHeader)?;

        let message_serialized_len = CanBusMessageEnum::serialized_len(header.message_type)
            .ok_or(DecodeAgregatedMessageError::InvalidMessage)?;
        if chunk.len() < 5 + message_serialized_len {
            return Err(DecodeAgregatedMessageError::ExpectMore(
                5 + message_serialized_len - chunk.len(),
            ));
        }
        let message = CanBusMessageEnum::deserialize(
            header.message_type,
            &chunk[5..(5 + message_serialized_len)],
        )
        .ok_or(DecodeAgregatedMessageError::InvalidMessage)?;

        Ok((
            5 + message_serialized_len,
            Self {
                node_type: header.node_type,
                node_id: header.node_id,
                message,
                count: header.count as usize,
            },
        ))
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum DecodeAgregatedMessageError {
    InvalidChunkHeader,
    InvalidMessageHeader,
    InvalidMessage,
    ExpectMore(usize),
    HeatshrinkError(HeatshrinkError),
}

/// returns is_overrun
pub fn decode_aggregated_can_bus_messages(
    chunk: &[u8],
    mut on_decoded_message: impl FnMut(DecodedMessage),
) -> Result<bool, DecodeAgregatedMessageError> {
    if chunk.len() < 3 {
        return Err(DecodeAgregatedMessageError::InvalidChunkHeader);
    }
    let chunk_header = ChunkHeader::deserialize(&chunk[..3])
        .ok_or(DecodeAgregatedMessageError::InvalidChunkHeader)?;
    let (compressed_chunk, mut uncompressed_chunk) = chunk[3..]
        .split_at_checked(chunk_header.compressed_len as usize)
        .ok_or(DecodeAgregatedMessageError::InvalidChunkHeader)?;

    // decompress, 5 is the size of message header
    let mut decompress_buffer = [0u8; { (5 + MAX_CAN_MESSAGE_SIZE) * 36 }];
    let mut dec: HeatshrinkWrapper<'_, HeatshrinkDecoder> =
        HeatshrinkWrapper::new(&mut decompress_buffer);
    dec.sink(compressed_chunk)
        .map_err(DecodeAgregatedMessageError::HeatshrinkError)?;
    let decompressed_len = dec
        .finish()
        .map_err(DecodeAgregatedMessageError::HeatshrinkError)?;
    // the lower the better
    log_info!(
        "{}B compressed + {}B uncompressed, compression ratio {}, total comperssion ratio {}",
        compressed_chunk.len(),
        uncompressed_chunk.len(),
        compressed_chunk.len() as f32 / decompressed_len as f32,
        chunk.len() as f32 / (decompressed_len + uncompressed_chunk.len()) as f32,
    );

    let mut decompress_buffer = &decompress_buffer[..decompressed_len];
    while decompress_buffer.len() > 0 {
        let (consumed_len, message) = DecodedMessage::decode(decompress_buffer)?;
        decompress_buffer = &decompress_buffer[consumed_len..];
        on_decoded_message(message);
    }
    while uncompressed_chunk.len() > 0 {
        let (consumed_len, message) = DecodedMessage::decode(uncompressed_chunk)?;
        uncompressed_chunk = &uncompressed_chunk[consumed_len..];
        on_decoded_message(message);
    }

    Ok(chunk_header.overrun)
}

#[cfg(test)]
mod test {
    use crate::{
        can_bus::{
            messages::{
                IMU_MEASUREMENT_MESSAGE_TYPE, NODE_STATUS_MESSAGE_TYPE,
                imu_measurement::IMUMeasurementMessage,
                node_status::{NodeHealth, NodeMode, NodeStatusMessage},
            },
            node_types::{AMP_NODE_TYPE, VOID_LAKE_NODE_TYPE},
        },
        tests::init_logger,
    };
    use log::info;

    use super::*;

    #[test]
    fn test_aggregate_message() {
        init_logger();

        let messages: std::vec::Vec<(CanBusExtendedId, u64, CanBusMessageEnum)> = vec![
            (
                CanBusExtendedId::new(5, IMU_MEASUREMENT_MESSAGE_TYPE, VOID_LAKE_NODE_TYPE, 1),
                1,
                CanBusMessageEnum::IMUMeasurement(IMUMeasurementMessage::new(
                    10000000,
                    &[1.5, 2.5, 3.5],
                    &[1.5, 2.5, 3.5],
                )),
            ),
            (
                CanBusExtendedId::new(5, NODE_STATUS_MESSAGE_TYPE, AMP_NODE_TYPE, 2),
                2,
                CanBusMessageEnum::NodeStatus(NodeStatusMessage {
                    uptime_s: 10,
                    health: NodeHealth::Healthy,
                    mode: NodeMode::Maintainance,
                    custom_status: 0,
                }),
            ),
            (
                CanBusExtendedId::new(5, IMU_MEASUREMENT_MESSAGE_TYPE, VOID_LAKE_NODE_TYPE, 1),
                3,
                CanBusMessageEnum::IMUMeasurement(IMUMeasurementMessage::new(
                    10000000,
                    &[10.5, 20.5, 30.5],
                    &[10.5, 20.5, 30.5],
                )),
            ),
        ];

        let mut aggregator = CanBusMessageAggregator::new();
        for (id, timestamp, message) in &messages {
            aggregator.process_message(id, message, *timestamp);
        }

        // create chunk
        let mut chunk = [0u8; 512];
        let chunk_len = aggregator.create_chunk(&mut chunk);
        info!("chunk_len: {}", chunk_len);
        assert!(aggregator.entries.is_empty());

        // decode the chunk
        let mut outputs = std::vec::Vec::<DecodedMessage>::new();
        decode_aggregated_can_bus_messages(&chunk[..chunk_len], |message| outputs.push(message))
            .unwrap();

        assert_eq!(outputs.len(), 2);
        assert_eq!(outputs[0].message, messages[2].2);
        assert_eq!(outputs[1].message, messages[1].2);
    }
}
