//! On-SD-card and over-USB storage format for flight data records.
//!
//! ```text
//! block 0            : superblock (see [`encode_superblock`])
//! block 1 .. 1+N     : tagged records packed back-to-back:
//!                      [tag:1][rkyv body] [tag:1][rkyv body] ...
//!                      zero-padded, CRC32 in the last 4 bytes.
//! ```
//!
//! Tags: [`RECORD_TAG_FAST`], [`RECORD_TAG_SLOW`] (see `flight_data_record`).
//!
//! Older layouts (v1 fixed records, v2/v3 tagged streams) are NOT readable:
//! the firmware starts a fresh log over them and rocket-cli reports a clean
//! "unsupported format" error instead of decoding.

use crate::flight_data_record::{
    FlightDataFastRecord, FlightDataSlowRecord, LogRecord, RECORD_TAG_FAST, RECORD_TAG_SLOW,
};

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

/// Identifies the avionics config block (last SD block; independent of the flight log).
pub const CONFIG_BLOCK_MAGIC: [u8; 4] = *b"VLFC";

/// On-disk config block format version.
pub const CONFIG_BLOCK_VERSION: u32 = 1;

/// Default target apogee AGL (m) when no config is stored.
pub const DEFAULT_TARGET_APOGEE_AGL: f32 = 4000.0;

/// On-disk format version. Bump when the record or superblock layout changes;
/// logs written at any other version are treated as absent.
/// v5: fast record gains the airbrakes estimator's altitude / vertical
/// velocity / tilt and its status flags (lockout votes, born, clip).
pub const STORAGE_VERSION: u32 = 5;

/// USB/superblock `record_len` field for the tagged stream (variable per record).
pub const RECORD_LEN_TAGGED: u32 = 0;

/// rkyv body sizes for tagged record types.
pub const FAST_BODY_LEN: usize = size_of::<<FlightDataFastRecord as rkyv::Archive>::Archived>();
pub const SLOW_BODY_LEN: usize = size_of::<<FlightDataSlowRecord as rkyv::Archive>::Archived>();

pub const FAST_WIRE_LEN: usize = 1 + FAST_BODY_LEN;
pub const SLOW_WIRE_LEN: usize = 1 + SLOW_BODY_LEN;

/// Largest tagged record on the wire.
pub const MAX_WIRE_LEN: usize = if FAST_WIRE_LEN > SLOW_WIRE_LEN {
    FAST_WIRE_LEN
} else {
    SLOW_WIRE_LEN
};

#[repr(C, align(16))]
struct AlignedBuf<const N: usize>([u8; N]);

fn crc32(data: &[u8]) -> u32 {
    crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC).checksum(data)
}

fn serialize_fast_body(fast: &FlightDataFastRecord) -> [u8; FAST_BODY_LEN] {
    let mut scratch = AlignedBuf([0u8; FAST_BODY_LEN]);
    to_bytes_in_with_alloc::<_, _, Failure>(
        fast,
        Buffer::from(&mut scratch.0[..]),
        SubAllocator::empty(),
    )
    .expect("FAST serialization cannot fail");
    scratch.0
}

fn serialize_slow_body(slow: &FlightDataSlowRecord) -> [u8; SLOW_BODY_LEN] {
    let mut scratch = AlignedBuf([0u8; SLOW_BODY_LEN]);
    to_bytes_in_with_alloc::<_, _, Failure>(
        slow,
        Buffer::from(&mut scratch.0[..]),
        SubAllocator::empty(),
    )
    .expect("SLOW serialization cannot fail");
    scratch.0
}

fn deserialize_fast_body(bytes: &[u8]) -> Option<FlightDataFastRecord> {
    if bytes.len() < FAST_BODY_LEN {
        return None;
    }
    let mut aligned = AlignedBuf([0u8; FAST_BODY_LEN]);
    aligned.0.copy_from_slice(&bytes[..FAST_BODY_LEN]);
    unsafe { from_bytes_unchecked::<FlightDataFastRecord, Failure>(&aligned.0) }.ok()
}

fn deserialize_slow_body(bytes: &[u8]) -> Option<FlightDataSlowRecord> {
    if bytes.len() < SLOW_BODY_LEN {
        return None;
    }
    let mut aligned = AlignedBuf([0u8; SLOW_BODY_LEN]);
    aligned.0.copy_from_slice(&bytes[..SLOW_BODY_LEN]);
    unsafe { from_bytes_unchecked::<FlightDataSlowRecord, Failure>(&aligned.0) }.ok()
}

/// Serialise a tagged record. Returns the wire bytes and their length.
pub fn serialize_log_record(record: &LogRecord) -> ([u8; MAX_WIRE_LEN], usize) {
    let mut buf = [0u8; MAX_WIRE_LEN];
    let len = match record {
        LogRecord::Fast(fast) => {
            buf[0] = RECORD_TAG_FAST;
            let body = serialize_fast_body(fast);
            buf[1..1 + FAST_BODY_LEN].copy_from_slice(&body);
            FAST_WIRE_LEN
        }
        LogRecord::Slow(slow) => {
            buf[0] = RECORD_TAG_SLOW;
            let body = serialize_slow_body(slow);
            buf[1..1 + SLOW_BODY_LEN].copy_from_slice(&body);
            SLOW_WIRE_LEN
        }
    };
    (buf, len)
}

/// Wire length of the tagged record starting at `bytes`, or `None` if unknown tag.
pub fn log_record_wire_len(bytes: &[u8]) -> Option<usize> {
    match *bytes.first()? {
        RECORD_TAG_FAST => Some(FAST_WIRE_LEN),
        RECORD_TAG_SLOW => Some(SLOW_WIRE_LEN),
        _ => None,
    }
}

/// Deserialise one tagged record from a block slice at `offset`.
pub fn deserialize_log_record_at(block: &[u8], offset: usize) -> Option<(LogRecord, usize)> {
    let wire_len = log_record_wire_len(&block[offset..])?;
    let end = offset + wire_len;
    if end > block.len() {
        return None;
    }
    let record = match block[offset] {
        RECORD_TAG_FAST => LogRecord::Fast(deserialize_fast_body(&block[offset + 1..end])?),
        RECORD_TAG_SLOW => LogRecord::Slow(deserialize_slow_body(&block[offset + 1..end])?),
        _ => return None,
    };
    Some((record, wire_len))
}

/// Count tagged records whose wire image fits in `data[..used_bytes]`.
pub fn count_records_in_bytes(data: &[u8], used_bytes: usize) -> u32 {
    let mut off = 0usize;
    let mut count = 0u32;
    let end = used_bytes.min(data.len());
    while off < end {
        let Some(wire_len) = log_record_wire_len(&data[off..end]) else {
            break;
        };
        if off + wire_len > end {
            break;
        }
        off += wire_len;
        count += 1;
    }
    count
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
    pub storage_version: u32,
    /// Total number of records in the log.
    pub record_count: u32,
    /// Number of live data blocks (starting at [`DATA_START_BLOCK`]).
    pub block_count: u32,
    /// Bytes used in the last data block.
    pub last_block_offset: u32,
}

/// Decoded contents of a valid avionics config block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AvionicsConfig {
    pub target_apogee_agl: f32,
}

impl Default for AvionicsConfig {
    fn default() -> Self {
        Self {
            target_apogee_agl: DEFAULT_TARGET_APOGEE_AGL,
        }
    }
}

/// Build a 512-byte config block (stored at the last SD block index).
///
/// Layout: magic(4) | version(4) | target_apogee_agl f32 LE(4) | reserved... | crc32(4).
pub fn encode_config_block(config: &AvionicsConfig) -> [u8; BLOCK_SIZE] {
    let mut b = [0u8; BLOCK_SIZE];
    b[0..4].copy_from_slice(&CONFIG_BLOCK_MAGIC);
    b[4..8].copy_from_slice(&CONFIG_BLOCK_VERSION.to_le_bytes());
    b[8..12].copy_from_slice(&config.target_apogee_agl.to_le_bytes());
    let crc = crc32(&b[..USABLE_PER_BLOCK]);
    b[USABLE_PER_BLOCK..].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Parse an avionics config block. Returns `None` if magic/CRC/version are invalid.
pub fn decode_config_block(block: &[u8; BLOCK_SIZE]) -> Option<AvionicsConfig> {
    if block[0..4] != CONFIG_BLOCK_MAGIC {
        return None;
    }
    if !verify_data_block(block) {
        return None;
    }
    let version = u32::from_le_bytes(block[4..8].try_into().ok()?);
    if version != CONFIG_BLOCK_VERSION {
        return None;
    }
    Some(AvionicsConfig {
        target_apogee_agl: f32::from_le_bytes(block[8..12].try_into().ok()?),
    })
}

/// Build a 512-byte superblock describing the current log state.
///
/// Layout: magic(4) | version(4) | record_count(4) | block_count(4) |
/// last_block_offset(4) | reserved(4) | crc32(4, last 4 bytes).
pub fn encode_superblock(record_count: u32, block_count: u32, last_block_offset: u32) -> [u8; BLOCK_SIZE] {
    let mut b = [0u8; BLOCK_SIZE];
    b[0..4].copy_from_slice(&SUPERBLOCK_MAGIC);
    b[4..8].copy_from_slice(&STORAGE_VERSION.to_le_bytes());
    b[8..12].copy_from_slice(&record_count.to_le_bytes());
    b[12..16].copy_from_slice(&block_count.to_le_bytes());
    b[16..20].copy_from_slice(&last_block_offset.to_le_bytes());
    let crc = crc32(&b[..USABLE_PER_BLOCK]);
    b[USABLE_PER_BLOCK..].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Parse a superblock. Superblocks from other storage versions decode to `None`
/// (the firmware then starts a fresh log over the old data).
pub fn decode_superblock(block: &[u8; BLOCK_SIZE]) -> Option<SuperblockInfo> {
    if block[0..4] != SUPERBLOCK_MAGIC {
        return None;
    }
    if !verify_data_block(block) {
        return None;
    }
    let version = u32::from_le_bytes(block[4..8].try_into().ok()?);
    if version != STORAGE_VERSION {
        return None;
    }
    Some(SuperblockInfo {
        storage_version: version,
        record_count: u32::from_le_bytes(block[8..12].try_into().ok()?),
        block_count: u32::from_le_bytes(block[12..16].try_into().ok()?),
        last_block_offset: u32::from_le_bytes(block[16..20].try_into().ok()?),
    })
}

/// Build the 16-byte USB download response header.
///
/// `record_len` is [`RECORD_LEN_TAGGED`] (0) for the tagged stream.
pub fn encode_response_header(record_count: u32, block_count: u32) -> [u8; HEADER_LEN] {
    let mut h = [0u8; HEADER_LEN];
    h[0..4].copy_from_slice(&RESPONSE_MAGIC);
    h[4..8].copy_from_slice(&record_count.to_le_bytes());
    h[8..12].copy_from_slice(&RECORD_LEN_TAGGED.to_le_bytes());
    h[12..16].copy_from_slice(&block_count.to_le_bytes());
    h
}

/// Identifies a valid USB download response header.
pub const RESPONSE_MAGIC: [u8; 4] = *b"VLDR";

/// Length of the USB download response header in bytes.
pub const HEADER_LEN: usize = 16;

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

/// Parse tagged records from block bytes. Host only. Returns `None` when the
/// stream does not decode cleanly (e.g. a log written by older firmware).
#[cfg(any(feature = "std", test))]
pub fn parse_log_records(
    record_count: u32,
    blocks: &[u8],
    block_count: u32,
) -> Option<std::vec::Vec<LogRecord>> {
    let mut records = std::vec::Vec::with_capacity(record_count as usize);
    let mut read = 0u32;
    for i in 0..block_count as usize {
        let start = i * BLOCK_SIZE;
        let block = blocks.get(start..start + BLOCK_SIZE)?;
        let mut off = 0usize;
        while read < record_count {
            let Some((rec, wire_len)) = deserialize_log_record_at(block, off) else {
                break;
            };
            if off + wire_len > USABLE_PER_BLOCK {
                break;
            }
            records.push(rec);
            off += wire_len;
            read += 1;
        }
    }
    if records.len() as u32 == record_count {
        Some(records)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_bus::messages::vl_status::FlightStage;
    use crate::flight_data_record::{
        VALID_BARO, VALID_BATTERY, VALID_GPS_FIX, VALID_IMU, merge_log_records,
    };

    fn sample_fast(i: u32) -> FlightDataFastRecord {
        FlightDataFastRecord {
            sequence: i,
            timestamp_us: i as u64 * 2400,
            acc: [i as f32, -1.5, 9.81],
            gyro: [0.1, 0.2, 0.3],
            temperature: 21.5,
            pressure: 101325.0 - i as f32,
            mag: [12.0, -34.0, 56.0],
            kf_altitude_asl: 271.5 + i as f32,
            kf_vertical_velocity: 0.25 * i as f32,
            ab_altitude_asl: 272.0 + i as f32,
            ab_vertical_velocity: 0.3 * i as f32,
            ab_tilt_deg: 5.0,
            ab_flags: 0,
            flight_stage: FlightStage::PoweredAscent,
            valid: VALID_IMU | VALID_BARO,
        }
    }

    fn sample_slow(i: u32) -> FlightDataSlowRecord {
        FlightDataSlowRecord {
            timestamp_us: i as u64 * 1_000_000,
            battery_voltage: 7.4,
            lat_lon: (37.421998, -122.084),
            altitude: 100.0 + i as f32,
            num_of_fixed_satalites: 9,
            hdop: 1.1,
            vdop: 2.2,
            pdop: 3.3,
            flight_stage: FlightStage::Armed,
            pyro_flags: 0b0000_0101,
            air_brakes_commanded_extension: 0.25,
            air_brakes_actual_extension: 0.2,
            valid: VALID_BATTERY | VALID_GPS_FIX,
        }
    }

    fn pack_log(records: &[LogRecord]) -> (Vec<[u8; BLOCK_SIZE]>, u32) {
        let mut blocks: Vec<[u8; BLOCK_SIZE]> = Vec::new();
        let mut cur = [0u8; BLOCK_SIZE];
        let mut off = 0usize;
        for r in records {
            let (bytes, len) = serialize_log_record(r);
            if off + len > USABLE_PER_BLOCK {
                let mut full = cur;
                finalize_data_block(&mut full);
                blocks.push(full);
                cur = [0u8; BLOCK_SIZE];
                off = 0;
            }
            cur[off..off + len].copy_from_slice(&bytes[..len]);
            off += len;
        }
        if off > 0 {
            let mut last = cur;
            finalize_data_block(&mut last);
            blocks.push(last);
        }
        (blocks, off as u32)
    }

    #[test]
    fn fast_record_round_trips() {
        let r = LogRecord::Fast(sample_fast(7));
        let (bytes, len) = serialize_log_record(&r);
        assert_eq!(len, FAST_WIRE_LEN);
        let (back, wire) = deserialize_log_record_at(&bytes[..len], 0).unwrap();
        assert_eq!(wire, FAST_WIRE_LEN);
        assert_eq!(back, r);
    }

    #[test]
    fn slow_record_round_trips() {
        let r = LogRecord::Slow(sample_slow(3));
        let (bytes, len) = serialize_log_record(&r);
        assert_eq!(len, SLOW_WIRE_LEN);
        let (back, _) = deserialize_log_record_at(&bytes[..len], 0).unwrap();
        assert_eq!(back, r);
    }

    #[test]
    fn superblock_round_trips() {
        let sb = encode_superblock(99, 5, 123);
        let info = decode_superblock(&sb).expect("decode");
        assert_eq!(info.storage_version, STORAGE_VERSION);
        assert_eq!(info.record_count, 99);
        assert_eq!(info.block_count, 5);
        assert_eq!(info.last_block_offset, 123);
    }

    #[test]
    fn old_version_superblock_rejected() {
        let mut sb = encode_superblock(99, 5, 123);
        sb[4..8].copy_from_slice(&3u32.to_le_bytes());
        let crc = super::crc32(&sb[..USABLE_PER_BLOCK]);
        sb[USABLE_PER_BLOCK..].copy_from_slice(&crc.to_le_bytes());
        assert!(decode_superblock(&sb).is_none());
    }

    #[test]
    fn config_block_round_trips() {
        let cfg = AvionicsConfig {
            target_apogee_agl: 3500.5,
        };
        let block = encode_config_block(&cfg);
        let back = decode_config_block(&block).expect("decode");
        assert_eq!(back.target_apogee_agl, 3500.5);
    }

    #[test]
    fn config_block_rejects_bad_magic() {
        let mut block = encode_config_block(&AvionicsConfig::default());
        block[0] = b'X';
        assert!(decode_config_block(&block).is_none());
    }

    #[test]
    fn tagged_download_round_trips() {
        let mut log: Vec<LogRecord> = Vec::new();
        for i in 0..20 {
            if i % 5 == 0 {
                log.push(LogRecord::Slow(sample_slow(i)));
            }
            log.push(LogRecord::Fast(sample_fast(i)));
        }
        let n = log.len() as u32;
        let (blocks, last_off) = pack_log(&log);

        let mut wire = Vec::new();
        wire.extend_from_slice(&encode_response_header(n, blocks.len() as u32));
        for b in &blocks {
            wire.extend_from_slice(b);
        }

        let (record_count, record_len, block_count) =
            decode_response_header(&wire).unwrap();
        assert_eq!(record_count, n);
        assert_eq!(record_len, RECORD_LEN_TAGGED);
        let recovered = parse_log_records(record_count, &wire[HEADER_LEN..], block_count).unwrap();
        assert_eq!(recovered, log);

        let merged = merge_log_records(&recovered);
        assert_eq!(merged.len(), 20);
        assert_eq!(merged[0].record_count, 0);
        // Stage comes from the fast record at full rate, not the slow snapshot.
        assert_eq!(merged[0].flight_stage, FlightStage::PoweredAscent);
        assert_eq!(merged[0].kf_altitude_asl, 271.5);

        let sb = encode_superblock(n, blocks.len() as u32, last_off);
        let info = decode_superblock(&sb).unwrap();
        assert_eq!(info.last_block_offset, last_off);
    }
}
