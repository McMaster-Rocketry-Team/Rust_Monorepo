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
/// v13: the airbrakes flag byte is renumbered. Bit 4 was held empty for the
///     retired `AIRBRAKES_APOGEE` latch so pre-2026-08-17 logs kept their
///     meaning; every board and host now runs current code, so the hole is
///     reclaimed — baro-resync 5 -> 4, pad-calibrated 6 -> 5, bits 0-3
///     unmoved. This bump is what keeps that safe: the two layouts would
///     otherwise both claim v12 and an old card would decode as valid with
///     three flags silently shifted, instead of being rejected here.
/// v12: absent data is `Option` everywhere instead of a sentinel value.
///     `f32::NAN`, `PAYLOAD_READING_UNAVAILABLE` (0xFFFF), a zero
///     `unix_time_us`, the `valid` bitmask and its `VALID_*` flags are all
///     gone; sources that go missing as a unit (IMU, the two estimators, the
///     AMP status, each node heartbeat) moved into grouped structs behind one
///     `Option` each. Records grew — a sentinel costs nothing, an `Option`
///     costs a discriminant plus padding — which is the price of a log that
///     cannot mistake a reading for its own absence.
/// v11: the slow record stores the full `NodeStatusMessage` (uptime, health,
///     mode, custom status) for AMP / Icarus / OZYS / payload SDRM, replacing
///     the lone `amp_online` bool. One OZYS this year, addressed by node type.
/// v10: airbrakes commanded + actual extension moved from the fast record to
///     the slow one (both only ever update at 10 Hz);
///     `air_brakes_validation_deploy` added to the slow record.
/// v9: baro `temperature` moved from the fast record to the slow one;
///     redundant `flight_stage` dropped from the slow record; estimator
///     fields renamed, `deployment_flags` added to the fast record,
///     `mpc_predicted_apogee_agl` added to the slow record, `VALID_BARO` dropped.
/// v8: payload EPM rail currents + SEM actuator steps in the slow record.
/// v7: tagged FAST/SLOW stream (see `flight_data_record`). Older formats: see git history.
pub const STORAGE_VERSION: u32 = 13;

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
/// Layout: magic(4) | record_count(4) | storage_version(4) | block_count(4).
pub fn encode_response_header(
    record_count: u32,
    storage_version: u32,
    block_count: u32,
) -> [u8; HEADER_LEN] {
    let mut h = [0u8; HEADER_LEN];
    h[0..4].copy_from_slice(&RESPONSE_MAGIC);
    h[4..8].copy_from_slice(&record_count.to_le_bytes());
    h[8..12].copy_from_slice(&storage_version.to_le_bytes());
    h[12..16].copy_from_slice(&block_count.to_le_bytes());
    h
}

/// Identifies a valid USB download response header.
pub const RESPONSE_MAGIC: [u8; 4] = *b"VLDR";

/// Length of the USB download response header in bytes.
pub const HEADER_LEN: usize = 16;

/// Decoded USB download response header: `(record_count, storage_version, block_count)`.
pub fn decode_response_header(buf: &[u8]) -> Option<(u32, u32, u32)> {
    if buf.len() < HEADER_LEN || buf[0..4] != RESPONSE_MAGIC {
        return None;
    }
    let record_count = u32::from_le_bytes(buf[4..8].try_into().ok()?);
    let storage_version = u32::from_le_bytes(buf[8..12].try_into().ok()?);
    let block_count = u32::from_le_bytes(buf[12..16].try_into().ok()?);
    Some((record_count, storage_version, block_count))
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
    use crate::can_bus::messages::node_status::{NodeHealth, NodeMode};
    use crate::flight_data_record::{
        AirBrakesRecord, AirbrakesEstimatorRecord, AmpRecord, DeploymentEstimatorRecord, ImuRecord,
        NodeStatusRecord, PayloadRecord, merge_log_records,
    };

    fn sample_fast(i: u32) -> FlightDataFastRecord {
        FlightDataFastRecord {
            sequence: i,
            timestamp_us: i as u64 * 2400,
            unix_time_us: Some(1_750_000_000_000_000 + i as u64 * 2400),
            imu: Some(ImuRecord {
                acc: [i as f32, -1.5, 9.81],
                gyro: [0.1, 0.2, 0.3],
            }),
            pressure: 101325.0 - i as f32,
            mag: Some([12.0, -34.0, 56.0]),
            deployment: Some(DeploymentEstimatorRecord {
                kf_altitude_asl: Some(271.5 + i as f32),
                kf_vertical_velocity: Some(0.25 * i as f32),
                flags: 0,
            }),
            airbrakes: Some(AirbrakesEstimatorRecord {
                kf_altitude_asl: Some(272.0 + i as f32),
                kf_vertical_velocity: Some(0.3 * i as f32),
                kf_tilt_deg: Some(5.0),
                flags: 0,
            }),
            flight_stage: FlightStage::Ascent,
            pyro_flags: Some(0b0000_0101),
        }
    }

    fn sample_slow(i: u32) -> FlightDataSlowRecord {
        FlightDataSlowRecord {
            timestamp_us: i as u64 * 1_000_000,
            temperature: 21.5,
            battery_voltage: Some(7.4),
            lat_lon: Some((37.421998, -122.084)),
            gps_altitude_asl: Some(100.0 + i as f32),
            num_of_fix_satellites: 9,
            hdop: Some(1.1),
            vdop: Some(2.2),
            pdop: Some(3.3),
            air_brakes: AirBrakesRecord {
                commanded_extension: Some(0.25),
                predicted_apogee_agl: Some(3010.0),
                validation_deploy: false,
                actual_extension: Some(0.2),
                servo_temp: Some(41.5),
            },
            amp: Some(AmpRecord {
                shared_battery_v: 8.2,
                out_status: 0b01_01_00,
            }),
            payload: PayloadRecord {
                epm_batt_mv: Some(12600),
                rail_ma: [Some(120), Some(340), None, Some(780), Some(1500), Some(2400)],
                actuator_steps: [Some(0), Some(1200), Some(34567)],
            },
            amp_node: Some(NodeStatusRecord {
                online: true,
                uptime_s: 42,
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                custom_status: 0,
            }),
            icarus_node: None,
            ozys_node: None,
            payload_sdrm_node: None,
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

    /// Absence has to survive the card, not just the type system.
    ///
    /// [`sample_fast`] populates every field, so on its own it only proves
    /// that *values* round trip — rkyv could drop an `Option`'s discriminant
    /// and every existing test would still pass. This is the shape the format
    /// exists to carry: a Mach-lockout sample, where the deployment filter is
    /// frozen and has nothing to report, riding next to an airbrakes half that
    /// is not born yet and an IMU that missed its tick. If any of these came
    /// back as `Some(0.0)`, a post-flight plot would show the rocket sitting
    /// at zero altitude through the fastest part of the flight.
    #[test]
    fn absent_fields_survive_the_round_trip() {
        let r = LogRecord::Fast(FlightDataFastRecord {
            deployment: Some(DeploymentEstimatorRecord {
                kf_altitude_asl: None,
                kf_vertical_velocity: None,
                flags: 0,
            }),
            airbrakes: None,
            imu: None,
            mag: None,
            unix_time_us: None,
            pyro_flags: None,
            ..sample_fast(7)
        });

        let (bytes, len) = serialize_log_record(&r);
        let (back, _) = deserialize_log_record_at(&bytes[..len], 0).unwrap();
        assert_eq!(back, r);

        let LogRecord::Fast(fast) = back else {
            panic!("tag changed across the round trip");
        };
        // Spelled out rather than left to the `assert_eq!` above, because the
        // failure this guards against is precisely an `Option` decaying into a
        // plausible-looking zero.
        let deployment = fast.deployment.expect("the estimator sample itself is present");
        assert_eq!(deployment.kf_altitude_asl, None);
        assert_eq!(deployment.kf_vertical_velocity, None);
        assert_eq!(fast.airbrakes, None);
        assert_eq!(fast.imu, None);
        assert_eq!(fast.mag, None);
        assert_eq!(fast.unix_time_us, None);
        assert_eq!(fast.pyro_flags, None);
        // The record is a fixed-width archive, so an all-absent sample costs
        // exactly what a fully populated one does.
        assert_eq!(len, FAST_WIRE_LEN);
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
        wire.extend_from_slice(&encode_response_header(n, STORAGE_VERSION, blocks.len() as u32));
        for b in &blocks {
            wire.extend_from_slice(b);
        }

        let (record_count, storage_version, block_count) =
            decode_response_header(&wire).unwrap();
        assert_eq!(record_count, n);
        assert_eq!(storage_version, STORAGE_VERSION);
        let recovered = parse_log_records(record_count, &wire[HEADER_LEN..], block_count).unwrap();
        assert_eq!(recovered, log);

        let merged = merge_log_records(&recovered);
        assert_eq!(merged.len(), 20);
        assert_eq!(merged[0].record_count, 0);
        // Stage and pyro flags come from the fast record at full rate, not
        // the slow snapshot; the AMP snapshot rides in the slow record.
        assert_eq!(merged[0].flight_stage, FlightStage::Ascent);
        assert_eq!(merged[0].pyro_flags, Some(0b0000_0101));
        assert_eq!(merged[0].unix_time_us, Some(1_750_000_000_000_000));
        assert!(merged[0].amp_node.as_ref().unwrap().online);
        // Nodes that never sent a heartbeat have no record at all.
        assert!(merged[0].ozys_node.is_none());
        assert_eq!(merged[0].amp.as_ref().unwrap().out_status, 0b01_01_00);
        assert_eq!(
            merged[0].deployment.as_ref().unwrap().kf_altitude_asl,
            Some(271.5)
        );

        let sb = encode_superblock(n, blocks.len() as u32, last_off);
        let info = decode_superblock(&sb).unwrap();
        assert_eq!(info.last_block_offset, last_off);
    }

    #[test]
    fn merge_before_first_slow_record_reports_nothing() {
        // A fast record logged before any slow snapshot has no slow data to
        // borrow, and says so rather than inventing a plausible zero.
        let merged = merge_log_records(&[LogRecord::Fast(sample_fast(0))]);
        assert_eq!(merged.len(), 1);
        assert!(merged[0].temperature.is_none());
        assert!(merged[0].num_of_fix_satellites.is_none());
        assert!(merged[0].air_brakes.is_none());
        assert!(merged[0].payload.is_none());
        assert!(merged[0].amp.is_none());
        // Fast-record columns are unaffected.
        assert_eq!(merged[0].pressure, 101325.0);

        let merged = merge_log_records(&[
            LogRecord::Slow(sample_slow(0)),
            LogRecord::Fast(sample_fast(0)),
        ]);
        assert_eq!(merged[0].temperature, Some(21.5));
        assert_eq!(merged[0].num_of_fix_satellites, Some(9));
        assert_eq!(merged[0].payload.as_ref().unwrap().rail_ma[2], None);
    }

}
