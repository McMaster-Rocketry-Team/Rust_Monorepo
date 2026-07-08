//! On-SD-card and over-USB storage format for flight data records.
//!
//! This module is the single source of truth shared by the **VLF5 firmware**
//! (which writes records to the SD card and streams them over USB) and the
//! **rocket-cli** host tool (which reads them back and writes CSV). Keeping the
//! layout here guarantees the two sides can never silently disagree.
//!
//! ## Layout on the SD card (raw 512-byte blocks, no filesystem)
//!
//! ```text
//! block 0            : superblock (see [`encode_superblock`])
//! block 1 .. 1+N     : N data blocks, each holding floor(508 / RECORD_LEN)
//!                      rkyv-serialised [`FlightDataRecord`]s, zero-padded, with
//!                      a CRC32 in the last 4 bytes.
//! ```
//!
//! Records never straddle a block boundary, so every block except the last one
//! is completely full. The superblock records how many records and how many
//! data blocks are live, so the log survives a power cycle.
//!
//! ## USB download protocol
//!
//! The host issues a vendor control transfer ([`crate::vlp::usb::CliRequest`]),
//! then reads the bulk-IN endpoint. The device replies with a
//! [`HEADER_LEN`]-byte response header (see [`encode_response_header`]) followed
//! by `block_count` raw 512-byte data blocks (for `Download`; `List`/`Clear`
//! send the header only), terminated by a zero-length packet.

use crate::flight_data_record::FlightDataRecord;

use rkyv::{
    api::low::{from_bytes_unchecked, to_bytes_in_with_alloc},
    rancor::Failure,
    ser::{allocator::SubAllocator, writer::Buffer},
};

/// Raw SD block size in bytes.
pub const BLOCK_SIZE: usize = 512;

/// Bytes of each data block usable for records. The trailing 4 bytes hold a
/// CRC32 over the rest of the block.
pub const USABLE_PER_BLOCK: usize = BLOCK_SIZE - 4;

/// Block index of the superblock.
pub const SUPERBLOCK_INDEX: u32 = 0;

/// Block index of the first data block.
pub const DATA_START_BLOCK: u32 = 1;

/// Identifies a valid superblock written by this firmware.
pub const SUPERBLOCK_MAGIC: [u8; 4] = *b"VLF5";

/// On-disk format version. Bump when the record or superblock layout changes.
pub const STORAGE_VERSION: u32 = 1;

/// Identifies a valid USB download response header.
pub const RESPONSE_MAGIC: [u8; 4] = *b"VLDR";

/// Length of the USB download response header in bytes.
pub const HEADER_LEN: usize = 16;

/// Serialised length of one [`FlightDataRecord`]. Computed from the rkyv
/// archived layout so the firmware and host always agree (both compile rkyv
/// with the same `pointer_width_32` feature).
pub const RECORD_LEN: usize = size_of::<<FlightDataRecord as rkyv::Archive>::Archived>();

/// Number of whole records that fit in one data block.
pub const RECORDS_PER_BLOCK: usize = USABLE_PER_BLOCK / RECORD_LEN;

/// rkyv needs its scratch/working buffer aligned; 16 covers every primitive in
/// [`FlightDataRecord`].
#[repr(C, align(16))]
struct AlignedRecord([u8; RECORD_LEN]);

fn crc32(data: &[u8]) -> u32 {
    crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC).checksum(data)
}

/// Serialise one record into an aligned `RECORD_LEN`-byte buffer.
pub fn serialize_record(record: &FlightDataRecord) -> [u8; RECORD_LEN] {
    let mut scratch = AlignedRecord([0u8; RECORD_LEN]);
    to_bytes_in_with_alloc::<_, _, Failure>(
        record,
        Buffer::from(&mut scratch.0[..]),
        SubAllocator::empty(),
    )
    .expect("record serialization cannot fail with a correctly-sized buffer");
    scratch.0
}

/// Deserialise one record from its `RECORD_LEN` leading bytes. Returns `None`
/// if the slice is too short.
///
/// # Safety note
/// Uses rkyv's unchecked path (matching the firmware's serialiser). The bytes
/// must have been produced by [`serialize_record`] for the same record layout.
pub fn deserialize_record(bytes: &[u8]) -> Option<FlightDataRecord> {
    if bytes.len() < RECORD_LEN {
        return None;
    }
    let mut aligned = AlignedRecord([0u8; RECORD_LEN]);
    aligned.0.copy_from_slice(&bytes[..RECORD_LEN]);
    // SAFETY: `aligned` is 16-byte aligned and contains a record produced by
    // `serialize_record`; FlightDataRecord is plain-old-data (no pointers).
    unsafe { from_bytes_unchecked::<FlightDataRecord, Failure>(&aligned.0) }.ok()
}

/// Stamp the CRC32 of `block[0..508]` into `block[508..512]`.
pub fn finalize_data_block(block: &mut [u8; BLOCK_SIZE]) {
    let crc = crc32(&block[..USABLE_PER_BLOCK]);
    block[USABLE_PER_BLOCK..].copy_from_slice(&crc.to_le_bytes());
}

/// Check the CRC32 trailer of a data block.
pub fn verify_data_block(block: &[u8; BLOCK_SIZE]) -> bool {
    let expected = crc32(&block[..USABLE_PER_BLOCK]);
    let stored = u32::from_le_bytes([
        block[USABLE_PER_BLOCK],
        block[USABLE_PER_BLOCK + 1],
        block[USABLE_PER_BLOCK + 2],
        block[USABLE_PER_BLOCK + 3],
    ]);
    expected == stored
}

/// Decoded contents of a valid superblock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SuperblockInfo {
    /// Total number of records in the log.
    pub record_count: u32,
    /// Number of live data blocks (starting at [`DATA_START_BLOCK`]).
    pub block_count: u32,
}

/// Build a 512-byte superblock describing the current log state.
///
/// Layout: magic(4) | version(4) | record_count(4) | block_count(4) |
/// record_len(4) | reserved | crc32(4, last 4 bytes).
pub fn encode_superblock(record_count: u32, block_count: u32) -> [u8; BLOCK_SIZE] {
    let mut b = [0u8; BLOCK_SIZE];
    b[0..4].copy_from_slice(&SUPERBLOCK_MAGIC);
    b[4..8].copy_from_slice(&STORAGE_VERSION.to_le_bytes());
    b[8..12].copy_from_slice(&record_count.to_le_bytes());
    b[12..16].copy_from_slice(&block_count.to_le_bytes());
    b[16..20].copy_from_slice(&(RECORD_LEN as u32).to_le_bytes());
    let crc = crc32(&b[..USABLE_PER_BLOCK]);
    b[USABLE_PER_BLOCK..].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Parse a superblock. Returns `None` if the magic, version, record length, or
/// CRC don't match what this build expects (e.g. an uninitialised or
/// incompatible card).
pub fn decode_superblock(block: &[u8; BLOCK_SIZE]) -> Option<SuperblockInfo> {
    if block[0..4] != SUPERBLOCK_MAGIC {
        return None;
    }
    if !verify_data_block(block) {
        return None;
    }
    let version = u32::from_le_bytes(block[4..8].try_into().ok()?);
    let record_len = u32::from_le_bytes(block[16..20].try_into().ok()?);
    if version != STORAGE_VERSION || record_len as usize != RECORD_LEN {
        return None;
    }
    Some(SuperblockInfo {
        record_count: u32::from_le_bytes(block[8..12].try_into().ok()?),
        block_count: u32::from_le_bytes(block[12..16].try_into().ok()?),
    })
}

/// Build the 16-byte USB download response header.
///
/// Layout: magic(4) | record_count(4) | record_len(4) | block_count(4).
pub fn encode_response_header(record_count: u32, block_count: u32) -> [u8; HEADER_LEN] {
    let mut h = [0u8; HEADER_LEN];
    h[0..4].copy_from_slice(&RESPONSE_MAGIC);
    h[4..8].copy_from_slice(&record_count.to_le_bytes());
    h[8..12].copy_from_slice(&(RECORD_LEN as u32).to_le_bytes());
    h[12..16].copy_from_slice(&block_count.to_le_bytes());
    h
}

/// Decoded USB download response header: `(record_count, record_len, block_count)`.
pub fn decode_response_header(buf: &[u8]) -> Option<(u32, u32, u32)> {
    if buf.len() < HEADER_LEN || buf[0..4] != RESPONSE_MAGIC {
        return None;
    }
    let record_count = u32::from_le_bytes(buf[4..8].try_into().ok()?);
    let record_len = u32::from_le_bytes(buf[8..12].try_into().ok()?);
    let block_count = u32::from_le_bytes(buf[12..16].try_into().ok()?);
    Some((record_count, record_len, block_count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_bus::messages::vl_status::FlightStage;
    use crate::flight_data_record::{VALID_BARO, VALID_GPS_FIX, VALID_IMU};

    fn sample(i: u32) -> FlightDataRecord {
        FlightDataRecord {
            record_count: i,
            timestamp_us: i as u64 * 2400,
            acc: [i as f32, -1.5, 9.81],
            gyro: [0.1, 0.2, 0.3],
            temperature: 21.5,
            pressure: 101325.0 - i as f32,
            mag: [12.0, -34.0, 56.0],
            battery_voltage: 7.4,
            valid: VALID_IMU | VALID_BARO | VALID_GPS_FIX,
            lat_lon: (37.421998, -122.084),
            altitude: 100.0 + i as f32,
            num_of_fixed_satalites: 9,
            hdop: 1.1,
            vdop: 2.2,
            pdop: 3.3,
            flight_stage: FlightStage::Armed,
            pyro_flags: 0b0000_0101,
        }
    }

    /// One record must survive serialize -> deserialize unchanged.
    #[test]
    fn record_round_trips() {
        let r = sample(42);
        let bytes = serialize_record(&r);
        assert_eq!(bytes.len(), RECORD_LEN);
        let back = deserialize_record(&bytes).expect("deserialize");
        assert_eq!(r, back);
    }

    #[test]
    fn data_block_crc() {
        let mut block = [7u8; BLOCK_SIZE];
        finalize_data_block(&mut block);
        assert!(verify_data_block(&block));
        block[10] ^= 0xFF;
        assert!(!verify_data_block(&block));
    }

    #[test]
    fn superblock_round_trips() {
        let sb = encode_superblock(1234, 56);
        let info = decode_superblock(&sb).expect("decode superblock");
        assert_eq!(info.record_count, 1234);
        assert_eq!(info.block_count, 56);
        // A flipped byte must fail the CRC.
        let mut bad = sb;
        bad[100] ^= 1;
        assert!(decode_superblock(&bad).is_none());
    }

    /// Full firmware-writer -> USB-stream -> host-parser round trip, including a
    /// partial final block, exactly mirroring `FlightLogger::append` (firmware)
    /// and `parse_records` (rocket-cli).
    #[test]
    fn full_download_round_trips() {
        // Enough records to fill several blocks with a partial tail.
        let n = RECORDS_PER_BLOCK as u32 * 3 + 2;
        let records: Vec<FlightDataRecord> = (0..n).map(sample).collect();

        // --- firmware side: pack records into 512-byte blocks ---
        let mut blocks: Vec<[u8; BLOCK_SIZE]> = Vec::new();
        let mut cur = [0u8; BLOCK_SIZE];
        let mut off = 0usize;
        for r in &records {
            let bytes = serialize_record(r);
            if off + bytes.len() > USABLE_PER_BLOCK {
                let mut full = cur;
                finalize_data_block(&mut full);
                blocks.push(full);
                cur = [0u8; BLOCK_SIZE];
                off = 0;
            }
            cur[off..off + bytes.len()].copy_from_slice(&bytes);
            off += bytes.len();
        }
        if off > 0 {
            let mut last = cur;
            finalize_data_block(&mut last);
            blocks.push(last);
        }

        // --- the wire: header followed by raw blocks ---
        let mut wire = Vec::new();
        wire.extend_from_slice(&encode_response_header(n, blocks.len() as u32));
        for b in &blocks {
            wire.extend_from_slice(b);
        }

        // --- host side: parse the stream back into records ---
        let (record_count, record_len, block_count) = decode_response_header(&wire).unwrap();
        assert_eq!(record_count, n);
        assert_eq!(record_len as usize, RECORD_LEN);
        assert_eq!(block_count as usize, blocks.len());

        let body = &wire[HEADER_LEN..];
        let mut recovered = Vec::new();
        let mut read = 0u32;
        for i in 0..block_count as usize {
            let block: &[u8; BLOCK_SIZE] =
                body[i * BLOCK_SIZE..(i + 1) * BLOCK_SIZE].try_into().unwrap();
            assert!(verify_data_block(block), "block {} failed CRC", i);
            let in_block = (RECORDS_PER_BLOCK as u32).min(record_count - read);
            for j in 0..in_block as usize {
                let o = j * RECORD_LEN;
                recovered.push(deserialize_record(&block[o..o + RECORD_LEN]).unwrap());
                read += 1;
            }
        }

        assert_eq!(recovered.len(), records.len());
        assert_eq!(recovered, records);
    }
}
