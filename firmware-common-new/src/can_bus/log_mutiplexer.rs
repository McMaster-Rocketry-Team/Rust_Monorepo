use super::{CanBusFrame, id::CanBusExtendedId, messages::LOG_MESSAGE_TYPE};
use heapless::{Deque, Vec};
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

pub struct LogMultiplexer {
    queue: Deque<(CanBusExtendedId, Vec<u8, 8>), 128>,
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

    pub fn create_chunk(&mut self, chunk: &mut [u8]) -> usize {
        let chunk_len = chunk.len();
        let mut offset = 0usize;
        let mut last_id: Option<CanBusExtendedId> = None;
        while chunk_len - offset >= 12 {
            if let Some((id, data)) = self.queue.pop_front() {
                if Some(id) == last_id {
                    let header = ThinHeader::new(data.len());
                    chunk[offset] = header.into();
                    offset += 1;
                } else {
                    let header = FullHeader::new(data.len(), id.node_type, id.node_id);
                    header.serialize(&mut chunk[offset..(offset + 4)]);
                    offset += 4;

                    last_id = Some(id);
                }

                chunk[offset..(offset + data.len())].copy_from_slice(&data);
                offset += data.len();
            } else {
                break;
            }
        }

        self.overrun = false;

        offset
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct DemultiplexedLogFrame {
    node_type: u8,
    node_id: u16,
    data: Vec<u8, 8>,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum DemultiplexLogError {
    ExpectMore(usize),
    HeaderLenOutOfRange(u8),
    InvalidFullHeader,
    OutputFull(usize),
}

pub fn demultiplex_log(
    mut chunk: &[u8],
    output: &mut Vec<DemultiplexedLogFrame, 128>,
) -> Result<(), DemultiplexLogError> {
    let mut last_type_id: Option<(u8, u16)> = None;

    let peek_thin_header = |chunk: &[u8]| -> Result<ThinHeader, DemultiplexLogError> {
        if chunk.len() == 0 {
            return Err(DemultiplexLogError::ExpectMore(1));
        }

        let header: ThinHeader = chunk[0].into();
        if header.len > 8 {
            return Err(DemultiplexLogError::HeaderLenOutOfRange(header.len));
        }

        Ok(header)
    };

    let consume_full_header = |chunk: &mut &[u8]| -> Result<FullHeader, DemultiplexLogError> {
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
    };

    let mut consume_data =
        |len: u8, chunk: &mut &[u8]| -> Result<Vec<u8, 8>, DemultiplexLogError> {
            let len = len as usize;
            if chunk.len() < len {
                return Err(DemultiplexLogError::ExpectMore(len - chunk.len()));
            }
            let mut data = Vec::<u8, 8>::new();
            data.extend_from_slice(&chunk[..len]).unwrap();
            *chunk = &chunk[len..];
            Ok(data)
        };

    while chunk.len() > 0 {
        if output.is_full() {
            return Err(DemultiplexLogError::OutputFull(chunk.len()));
        }
        if let Some((last_node_type, last_node_id)) = last_type_id {
            let header = peek_thin_header(chunk)?;
            if header.is_continue {
                chunk = &chunk[1..];

                output
                    .push(DemultiplexedLogFrame {
                        node_type: last_node_type,
                        node_id: last_node_id,
                        data: consume_data(header.len, &mut chunk)?,
                    })
                    .unwrap();
                continue;
            }
        }

        let header = consume_full_header(&mut chunk)?;
        last_type_id = Some((header.node_type, header.node_id));

        output
            .push(DemultiplexedLogFrame {
                node_type: header.node_type,
                node_id: header.node_id,
                data: consume_data(header.len, &mut chunk)?,
            })
            .unwrap();
    }

    Ok(())
}

#[cfg(test)]
mod test {
    use log::info;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_multiplex_log() {
        init_logger();

        let mut multiplexer = LogMultiplexer::new();

        // Create test frames with different IDs and data
        let frame1 = (
            0,
            CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 1, 1).into(),
            &[1, 2, 3],
        );
        let frame2 = (
            0,
            CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 1, 1).into(),
            &[4, 5, 6],
        );
        let frame3 = (
            0,
            CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 2, 2).into(),
            &[7, 8],
        );
        let frame4 = (
            0,
            CanBusExtendedId::new(7, LOG_MESSAGE_TYPE, 3, 3).into(),
            &[9, 10, 11, 12],
        );

        // Process all frames
        multiplexer.process_frame(&frame1);
        multiplexer.process_frame(&frame2);
        multiplexer.process_frame(&frame3);
        multiplexer.process_frame(&frame4);

        // Create a chunk to hold the multiplexed data
        let mut chunk = [0u8; 64];
        let chunk_len = multiplexer.create_chunk(&mut chunk);
        info!("chunk_len: {}", chunk_len);

        // Demultiplex the chunk
        let mut output = Vec::<DemultiplexedLogFrame, 128>::new();
        demultiplex_log(&chunk[..chunk_len], &mut output).unwrap();

        // Verify the results
        assert_eq!(output.len(), 4);

        // Check first frame
        assert_eq!(output[0].node_type, 1);
        assert_eq!(output[0].node_id, 1);
        assert_eq!(output[0].data.as_slice(), &[1, 2, 3]);

        // Check second frame (same ID as first)
        assert_eq!(output[1].node_type, 1);
        assert_eq!(output[1].node_id, 1);
        assert_eq!(output[1].data.as_slice(), &[4, 5, 6]);

        // Check third frame
        assert_eq!(output[2].node_type, 2);
        assert_eq!(output[2].node_id, 2);
        assert_eq!(output[2].data.as_slice(), &[7, 8]);

        // Check fourth frame
        assert_eq!(output[3].node_type, 3);
        assert_eq!(output[3].node_id, 3);
        assert_eq!(output[3].data.as_slice(), &[9, 10, 11, 12]);
    }
}
