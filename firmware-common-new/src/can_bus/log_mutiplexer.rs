use core::convert::Infallible;

use crate::heatshrink::{HeatshrinkError, HeatshrinkWrapper};

use super::{CanBusFrame, id::CanBusExtendedId, messages::LOG_MESSAGE_TYPE};
use heapless::{Deque, Vec};
use heatshrink::{decoder::HeatshrinkDecoder, encoder::HeatshrinkEncoder};
use packed_struct::prelude::*;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", bit_numbering = "msb0", size_bytes = "1")]
#[repr(C)]
struct ThinHeader {
    is_continue: bool,
    #[packed_field(element_size_bits = "7")]
    len: u8,
}

impl ThinHeader {
    fn new(len: usize) -> Self {
        Self {
            is_continue: true,
            len: len as u8,
        }
    }
}

impl Into<u8> for ThinHeader {
    fn into(self) -> u8 {
        self.pack().unwrap()[0]
    }
}

impl From<u8> for ThinHeader {
    fn from(value: u8) -> Self {
        Self::unpack_from_slice(&[value]).unwrap()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", bit_numbering = "msb0", size_bytes = "4")]
#[repr(C)]
struct FullHeader {
    is_continue: ReservedZero<packed_bits::Bits<1>>,
    #[packed_field(element_size_bits = "7")]
    len: u8,
    node_type: u8,
    node_id: u16,
}

impl FullHeader {
    fn new(len: usize, node_type: u8, node_id: u16) -> Self {
        Self {
            is_continue: Default::default(),
            len: len as u8,
            node_type,
            node_id,
        }
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..4]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", bit_numbering = "msb0", size_bytes = "3")]
#[repr(C)]
struct ChunkHeader {
    // the two reserved zeros indicate this chunk is a log multiplexer chunk
    _reserved: ReservedZero<packed_bits::Bits<2>>,
    overrun: bool,
    #[packed_field(element_size_bits = "10")]
    compressed_len: u16,
    #[packed_field(element_size_bits = "10")]
    uncompressed_len: u16,
}

impl ChunkHeader {
    fn new(overrun: bool, compressed_len: usize, uncompressed_len: usize) -> Self {
        Self {
            _reserved: Default::default(),
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

#[derive(Debug, Default)]
pub struct LogFrameEncoder {
    last_id: Option<CanBusExtendedId>,
    count: usize,
}

impl LogFrameEncoder {
    fn encode<E, F: FnMut(&[u8]) -> Result<(), E>>(
        &mut self,
        (id, data): &(CanBusExtendedId, Vec<u8, 8>),
        mut sink: F,
    ) -> Result<(), E> {
        if Some(id) == self.last_id.as_ref() {
            let header = ThinHeader::new(data.len());
            sink(&[header.into()])?;
        } else {
            let header = FullHeader::new(data.len(), id.node_type, id.node_id);
            let mut buffer = [0u8; 4];
            header.serialize(&mut buffer);
            sink(&buffer)?;
            self.last_id = Some(id.clone());
        }
        sink(&data)?;
        self.count += 1;

        Ok(())
    }
}

const LOG_MULTIPLEXER_QUEUE_SIZE: usize = 128;

pub struct LogMultiplexer {
    queue: Deque<(CanBusExtendedId, Vec<u8, 8>), LOG_MULTIPLEXER_QUEUE_SIZE>,
    overrun: bool,
}

impl LogMultiplexer {
    pub fn new() -> Self {
        return LogMultiplexer {
            queue: Deque::new(),
            overrun: false,
        };
    }

    pub fn process_frame(&mut self, frame: &impl CanBusFrame) {
        let id = CanBusExtendedId::from_raw(frame.id());
        if id.message_type != LOG_MESSAGE_TYPE {
            return;
        }

        let mut data = Vec::<u8, 8>::new();
        data.extend_from_slice(frame.data())
            .expect("can frame should never exceed 8 bytes");
        if self.queue.is_full() {
            self.overrun = true;
            self.queue.pop_front();
        }
        self.queue.push_back((id, data)).unwrap();
    }

    // compress at most chunk.len() length of data
    // does not pop frames out of queue
    // returns (frames_compressed, compressed_len)
    // errors if compressed_len > chunk.len()
    fn compress_at_most_chunk_len(
        &self,
        log_frame_enc: &mut LogFrameEncoder,
        chunk: &mut [u8],
    ) -> Result<(usize, usize), HeatshrinkError> {
        let mut hs_enc: HeatshrinkWrapper<'_, HeatshrinkEncoder> = HeatshrinkWrapper::new(chunk);
        let mut frame_iter = self.queue.iter();

        while hs_enc.in_len() + 12 <= hs_enc.out_buffer_len() {
            if let Some(frame) = frame_iter.next() {
                log_frame_enc.encode(frame, |data| hs_enc.sink(data))?;
            } else {
                break;
            }
        }

        let compressed_len = hs_enc.finish()?;
        Ok((log_frame_enc.count, compressed_len))
    }

    pub fn create_chunk(&mut self, chunk: &mut [u8]) -> usize {
        // to ensure compressed data always fit in the provided chunk in one pass,
        // the following algorithm is used:
        // 1. compress at most chunk.len() length of data using heatshrink
        // 2. if compressed length is the same as chunk length, use the compressed data
        // 3. if compressed length is longer than original length, dispose the compressed
        //    data and fill chunk with original data
        // 4. if compressed length is shorter than chunk length, fill the unused space with
        //    uncompressed data
        if chunk.len() <= 3 || self.queue.is_empty() {
            return 0;
        }
        let (header_buffer, chunk) = chunk.split_at_mut(3);
        let mut log_frame_enc = LogFrameEncoder::default();

        let compressed_len = if let Ok((frames_compressed, compressed_len)) =
            self.compress_at_most_chunk_len(&mut log_frame_enc, chunk)
        {
            // compressed_len <= chunk.len()

            // pop all compressed frames
            for _ in 0..frames_compressed {
                self.queue.pop_front();
            }

            compressed_len
        } else {
            // compressed_len > chunk.len()
            0
        };

        // fill the free space left with uncompressed data
        let chunk = &mut chunk[compressed_len..];
        let mut offset = 0;
        while offset + 12 <= chunk.len() {
            if let Some(frame) = self.queue.pop_front() {
                log_frame_enc
                    .encode(&frame, |data| {
                        chunk[offset..(offset + data.len())].copy_from_slice(data);
                        offset += data.len();
                        Ok::<(), Infallible>(())
                    })
                    .unwrap();
            } else {
                break;
            }
        }

        let header = ChunkHeader::new(self.overrun, compressed_len, offset);
        header.serialize(header_buffer);
        self.overrun = false;

        3 + compressed_len + offset
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct DemultiplexedLogFrame {
    pub node_type: u8,
    pub node_id: u16,
    pub data: Vec<u8, 8>,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum DemultiplexLogError {
    InvalidChunkHeader,
    ExpectMore(usize),
    HeaderLenOutOfRange(u8),
    InvalidFullHeader,
    HeatshrinkError(HeatshrinkError),
}

#[derive(Debug)]
pub struct LogFrameDecoder<F: FnMut(DemultiplexedLogFrame)> {
    last_type_id: Option<(u8, u16)>,
    on_decoded_frame: F,
}

impl<F: FnMut(DemultiplexedLogFrame)> LogFrameDecoder<F> {
    fn new(on_decoded_frame: F) -> Self {
        Self {
            last_type_id: None,
            on_decoded_frame,
        }
    }

    fn peek_thin_header(chunk: &[u8]) -> Result<ThinHeader, DemultiplexLogError> {
        if chunk.len() == 0 {
            return Err(DemultiplexLogError::ExpectMore(1));
        }

        let header: ThinHeader = chunk[0].into();
        if header.len > 8 {
            return Err(DemultiplexLogError::HeaderLenOutOfRange(header.len));
        }

        Ok(header)
    }

    fn consume_full_header(chunk: &mut &[u8]) -> Result<FullHeader, DemultiplexLogError> {
        if chunk.len() < 4 {
            return Err(DemultiplexLogError::ExpectMore(4 - chunk.len()));
        }

        let header = if let Some(header) = FullHeader::deserialize(&chunk[..4]) {
            header
        } else {
            return Err(DemultiplexLogError::InvalidFullHeader);
        };
        *chunk = &chunk[4..];
        if header.len > 8 {
            return Err(DemultiplexLogError::HeaderLenOutOfRange(header.len));
        }

        Ok(header)
    }

    fn consume_data(len: u8, chunk: &mut &[u8]) -> Result<Vec<u8, 8>, DemultiplexLogError> {
        let len = len as usize;
        if chunk.len() < len {
            return Err(DemultiplexLogError::ExpectMore(len - chunk.len()));
        }
        let mut data = Vec::<u8, 8>::new();
        data.extend_from_slice(&chunk[..len]).unwrap();
        *chunk = &chunk[len..];
        Ok(data)
    }

    fn decode(&mut self, mut chunk: &[u8]) -> Result<(), DemultiplexLogError> {
        while chunk.len() > 0 {
            if let Some((last_node_type, last_node_id)) = self.last_type_id {
                let header = Self::peek_thin_header(chunk)?;
                if header.is_continue {
                    chunk = &chunk[1..];

                    (self.on_decoded_frame)(DemultiplexedLogFrame {
                        node_type: last_node_type,
                        node_id: last_node_id,
                        data: Self::consume_data(header.len, &mut chunk)?,
                    });
                    continue;
                }
            }

            let header = Self::consume_full_header(&mut chunk)?;
            self.last_type_id = Some((header.node_type, header.node_id));

            (self.on_decoded_frame)(DemultiplexedLogFrame {
                node_type: header.node_type,
                node_id: header.node_id,
                data: Self::consume_data(header.len, &mut chunk)?,
            });
        }
        Ok(())
    }
}

/// returns is_overrun
pub fn decode_multiplexed_log_chunk(
    chunk: &[u8],
    on_decoded_frame: impl FnMut(DemultiplexedLogFrame),
) -> Result<bool, DemultiplexLogError> {
    // the most data multiplexer can put in one chunk is LOG_MULTIPLEXER_QUEUE_SIZE frames, limited by the multiplexer queue
    // worst case each frame has a 4 byte full header + 8 byte data, total LOG_MULTIPLEXER_QUEUE_SIZE * 12 bytes
    // this is not optimized for memory, ideally we won't need decode_buffer, instead feeding outputs from
    // the heatshrink decoder directly into consume_full_header etc.
    // but this code only runs on a pc, so memory usage doesn't matter.

    if chunk.len() < 3 {
        return Err(DemultiplexLogError::InvalidChunkHeader);
    }
    let chunk_header =
        ChunkHeader::deserialize(&chunk[..3]).ok_or(DemultiplexLogError::InvalidChunkHeader)?;
    log_info!("chunk_header {:?}", chunk_header);
    let (compressed_chunk, uncompressed_chunk) = chunk[3..]
        .split_at_checked(chunk_header.compressed_len as usize)
        .ok_or(DemultiplexLogError::InvalidChunkHeader)?;

    // decompress
    let mut decompress_buffer = [0u8; { LOG_MULTIPLEXER_QUEUE_SIZE * 12 }];
    let mut dec: HeatshrinkWrapper<'_, HeatshrinkDecoder> =
        HeatshrinkWrapper::new(&mut decompress_buffer);
    dec.sink(compressed_chunk)
        .map_err(DemultiplexLogError::HeatshrinkError)?;
    let decompressed_len = dec.finish().map_err(DemultiplexLogError::HeatshrinkError)?;
    // the lower the better
    log_info!(
        "{}B compressed + {}B uncompressed, compression ratio {}, total comperssion ratio {}",
        compressed_chunk.len(),
        uncompressed_chunk.len(),
        compressed_chunk.len() as f32 / decompressed_len as f32,
        chunk.len() as f32 / (decompressed_len + uncompressed_chunk.len()) as f32,
    );

    // decode log frames
    let mut log_frame_decoder = LogFrameDecoder::new(on_decoded_frame);
    log_frame_decoder.decode(&decompress_buffer[..decompressed_len])?;
    log_frame_decoder.decode(uncompressed_chunk)?;

    Ok(chunk_header.overrun)
}

#[cfg(test)]
mod test {
    use crate::tests::init_logger;
    use lipsum::lipsum;
    use log::info;

    use super::*;

    #[test]
    fn test_multiplex_log() {
        init_logger();

        // Create test frames with different IDs and data
        let frames: std::vec::Vec<(u64, CanBusExtendedId, std::vec::Vec<u8>)> = vec![
            (
                0,
                CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 1, 1),
                vec![1, 2, 3],
            ),
            (
                0,
                CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 1, 1),
                vec![4, 5, 6],
            ),
            (
                0,
                CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 2, 2),
                vec![7, 8],
            ),
            (
                0,
                CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 3, 3),
                vec![9, 10, 11, 12],
            ),
        ];

        let mut multiplexer = LogMultiplexer::new();
        for frame in &frames {
            multiplexer.process_frame(&(frame.0, frame.1.into(), frame.2.as_slice()));
        }

        // Create a chunk to hold the multiplexed data
        let mut chunk = [0u8; 512];
        let chunk_len = multiplexer.create_chunk(&mut chunk);
        info!("chunk_len: {}", chunk_len);
        assert_eq!(multiplexer.queue.len(), 0);
        assert_eq!(multiplexer.overrun, false);

        // Decode the chunk
        let mut outputs = std::vec::Vec::<DemultiplexedLogFrame>::new();
        decode_multiplexed_log_chunk(&chunk[..chunk_len], |frame| outputs.push(frame)).unwrap();

        // Verify the results
        assert_eq!(outputs.len(), frames.len());

        for (i, frame) in frames.iter().enumerate() {
            let output = &outputs[i];
            assert_eq!(output.node_type, frame.1.node_type);
            assert_eq!(output.node_id, frame.1.node_id);
            assert_eq!(output.data.as_slice(), frame.2.as_slice());
        }
    }

    #[test]
    fn test_multiplex_log_full() {
        init_logger();

        let mut frames: std::vec::Vec<(u64, CanBusExtendedId, &[u8])> = vec![];

        let lip = lipsum(256);
        let mut data = lip.as_bytes();
        for i in 0..128 {
            let frame: (u64, CanBusExtendedId, &[u8]) = (
                i,
                CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 1, 1),
                &data[..8],
            );
            data = &data[8..];
            frames.push(frame);
        }

        let mut multiplexer = LogMultiplexer::new();
        for frame in &frames {
            multiplexer.process_frame(&(frame.0, frame.1.into(), frame.2));
        }

        // Create a chunk to hold the multiplexed data
        let mut chunk = [0u8; 512];
        let chunk_len = multiplexer.create_chunk(&mut chunk);
        info!(
            "chunk_len: {}, queue left: {}",
            chunk_len,
            multiplexer.queue.len()
        );
        assert_eq!(multiplexer.queue.len(), 63);
        assert_eq!(multiplexer.overrun, false);

        // Decode the chunk
        let mut outputs = std::vec::Vec::<DemultiplexedLogFrame>::new();
        decode_multiplexed_log_chunk(&chunk[..chunk_len], |frame| outputs.push(frame)).unwrap();

        assert_eq!(outputs.len(), frames.len() - multiplexer.queue.len());
        for (i, frame) in frames.iter().enumerate().take(outputs.len()) {
            let output = &outputs[i];
            assert_eq!(output.node_type, frame.1.node_type);
            assert_eq!(output.node_id, frame.1.node_id);
            assert_eq!(output.data.as_slice(), frame.2);
        }
    }
}
