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
#[cfg(any(feature = "std", test))]
use crate::flight_data_record::ParsedLogRecord;

use rkyv::{
    api::low::to_bytes_in_with_alloc,
    rancor::Failure,
    ser::{allocator::SubAllocator, writer::Buffer},
};

/// Host decode goes through rkyv's checked API; see [`deserialize_fast_body`].
#[cfg(feature = "std")]
use rkyv::api::low::from_bytes;
#[cfg(not(feature = "std"))]
use rkyv::api::low::from_bytes_unchecked;

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
/// v19: the airbrakes commanded + actual extension and the validation-deploy
///     flag move from the slow record back to the fast one, reversing v10.
///     v10 was right about the rates and wrong about what was being measured:
///     neither value moves faster than 100 Hz, but on a 10 Hz row each of
///     their edges was quantised by ~100 ms and the pair could not be read as
///     a step response. Icarus now reports every cycle of its 100 Hz control
///     loop (it was measuring at 100 Hz and sending one in ten), so the log
///     resolves command-to-servo latency. The fast record grows 16 B to 160 B
///     and still packs three to a block; the slow record loses 24 B and now
///     packs two, so the SD block rate goes slightly DOWN, not up.
/// v18: the airbrakes estimator's state is logged outright, as a two-bit
///     `AirbrakesState` in the top of the airbrakes flags byte. The
///     `AIRBRAKES_ENABLED` bit it replaces was true in exactly one state and
///     so duplicated `kf_altitude_asl.is_some()`; the state also separates
///     `Armed` from `Stage1`, which nothing in the log could do before.
/// v17: the airbrakes estimator is four one-way states (armed, ignition,
///     airbrakes enabled), so `AIRBRAKES_BARO_TRUSTED` and
///     `AIRBRAKES_MPC_PERMITTED` are the same fact and collapse into
///     `AIRBRAKES_ENABLED` (the same bit 3). The Mach limit moved from a
///     per-sample gate downstream to a condition of entering the last state,
///     which is what makes the two identical.
/// v16: `AIRBRAKES_MPC_PERMITTED` added to the airbrakes estimator flags —
///     the MPC's own run/stop gate, per sample. It occupies a bit that was
///     already there, so no record grew; the bump is because a v15 reader
///     would report the bit clear on every row rather than absent, which is a
///     wrong answer rather than a missing one.
/// v15: every altitude in the log is ASL, and the slow record carries the
///     launch pad altitude to convert them with. `launch_pad_altitude_asl`
///     added; `air_brakes_predicted_apogee_agl` and
///     `air_brakes_target_apogee_agl` become `..._asl`. The log stored two
///     AGL numbers and never the reference they were measured from, so
///     nothing offline could reproduce the AGL the firmware actually flew
///     on, or put those two columns on the same axis as the estimator and
///     GPS altitudes beside them. The target additionally changes source: it
///     is now the value the MPC latched at construction rather than a
///     per-record sample of the operator's target watch, which could drift
///     from it mid-flight.
/// v14: `air_brakes_target_apogee_agl` added to the slow record. The log
///     carried the MPC's prediction and its command but not the target they
///     were computed against, which lives only in the SD config block and the
///     live downlink — so a finished log could not distinguish a controller
///     that failed to reach a reachable target from one correctly declining to
///     chase an unreachable one.
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
pub const STORAGE_VERSION: u32 = 19;

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

/// Decode one FAST body.
///
/// # Why this is split by feature
///
/// The archived record is full of `#[repr(u8)]` discriminants: an
/// `ArchivedOption` tag is valid only as 0 or 1, `NodeHealth` / `NodeMode` /
/// `FlightStage` only over their listed values, and a `bool` only as 0 or 1.
/// rkyv's unchecked deserialize reads those bytes and `match`es on them
/// directly, so a single wrong byte is undefined behaviour, not a decode error.
///
/// That is not hypothetical here: it is the ordinary outcome of a block that
/// failed its CRC. Records are fixed width, so a corrupt body byte never
/// desynchronizes the stream — and with only ~4-5 tag bytes per 508-byte block,
/// well over 99% of single-byte corruptions land in a body and would otherwise
/// "decode" straight into that `match`.
///
/// On the host (`std`, which turns on `rkyv/bytecheck`) the bytes therefore go
/// through rkyv's checked API, which validates every discriminant before
/// anything reads them; a bad record comes back as `None` and the caller skips
/// it. The firmware only ever writes records — it never reads a card back — so
/// it keeps the unchecked path rather than carrying validation code it cannot
/// use.
#[cfg(feature = "std")]
fn deserialize_fast_body(bytes: &[u8]) -> Option<FlightDataFastRecord> {
    if bytes.len() < FAST_BODY_LEN {
        return None;
    }
    let mut aligned = AlignedBuf([0u8; FAST_BODY_LEN]);
    aligned.0.copy_from_slice(&bytes[..FAST_BODY_LEN]);
    from_bytes::<FlightDataFastRecord, Failure>(&aligned.0).ok()
}

/// See [`deserialize_fast_body`] for why the host and firmware paths differ.
#[cfg(feature = "std")]
fn deserialize_slow_body(bytes: &[u8]) -> Option<FlightDataSlowRecord> {
    if bytes.len() < SLOW_BODY_LEN {
        return None;
    }
    let mut aligned = AlignedBuf([0u8; SLOW_BODY_LEN]);
    aligned.0.copy_from_slice(&bytes[..SLOW_BODY_LEN]);
    from_bytes::<FlightDataSlowRecord, Failure>(&aligned.0).ok()
}

/// Firmware path: no validation. See [`deserialize_fast_body`].
///
/// Safe only against bytes this firmware itself just serialised. Anything that
/// decodes a card whose CRC has failed must use the `std` build.
#[cfg(not(feature = "std"))]
fn deserialize_fast_body(bytes: &[u8]) -> Option<FlightDataFastRecord> {
    if bytes.len() < FAST_BODY_LEN {
        return None;
    }
    let mut aligned = AlignedBuf([0u8; FAST_BODY_LEN]);
    aligned.0.copy_from_slice(&bytes[..FAST_BODY_LEN]);
    unsafe { from_bytes_unchecked::<FlightDataFastRecord, Failure>(&aligned.0) }.ok()
}

/// Firmware path: no validation. See [`deserialize_fast_body`].
#[cfg(not(feature = "std"))]
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
///
/// `None` means the record did not decode: an unknown tag, a truncated body,
/// or — on the host, where bodies are validated — a discriminant byte that is
/// not a value its type can hold. The record's wire length is still recoverable
/// from the tag via [`log_record_wire_len`], so a caller can skip a rejected
/// record and keep parsing the rest of the block.
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

/// Everything one downloaded block stream decodes to.
#[cfg(any(feature = "std", test))]
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedLog {
    /// The records, each tagged with whether its source block passed CRC.
    pub records: std::vec::Vec<ParsedLogRecord>,
    /// Data blocks whose CRC32 trailer did not match. Their records are still
    /// in `records`, marked.
    pub crc_failed_blocks: u32,
    /// Records whose archived body failed validation and were skipped rather
    /// than deserialised. Only a corrupt block can produce these.
    pub invalid_records: u32,
}

/// Parse tagged records from block bytes. Host only. Returns `None` when the
/// stream does not decode cleanly (e.g. a log written by older firmware).
///
/// A block that fails its CRC is *not* fatal: its records are parsed and
/// returned with `block_crc_ok: false` so the caller can mark them, because one
/// bad block must not make an otherwise good flight log unrecoverable. A record
/// whose body fails validation is skipped and counted — records are fixed
/// width, so the parser knows where the next one starts and the rest of the
/// block survives.
#[cfg(any(feature = "std", test))]
pub fn parse_log_records(record_count: u32, blocks: &[u8], block_count: u32) -> Option<ParsedLog> {
    let mut records = std::vec::Vec::with_capacity(record_count as usize);
    let mut crc_failed_blocks = 0u32;
    let mut invalid_records = 0u32;
    let mut read = 0u32;
    for i in 0..block_count as usize {
        let start = i * BLOCK_SIZE;
        let block: &[u8; BLOCK_SIZE] = blocks.get(start..start + BLOCK_SIZE)?.try_into().ok()?;
        let block_crc_ok = verify_data_block(block);
        if !block_crc_ok {
            crc_failed_blocks += 1;
        }
        let mut off = 0usize;
        while read < record_count {
            let Some(wire_len) = log_record_wire_len(&block[off..]) else {
                break;
            };
            if off + wire_len > USABLE_PER_BLOCK {
                break;
            }
            match deserialize_log_record_at(block, off) {
                Some((record, _)) => records.push(ParsedLogRecord {
                    record,
                    block_crc_ok,
                }),
                None => invalid_records += 1,
            }
            off += wire_len;
            read += 1;
        }
    }
    if read == record_count {
        Some(ParsedLog {
            records,
            crc_failed_blocks,
            invalid_records,
        })
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
        AirBrakesActuationRecord, AirBrakesRecord, AirbrakesEstimatorRecord, AmpRecord,
        DeploymentEstimatorRecord, ImuRecord, NodeStatusRecord, ParsedLogRecord, PayloadRecord,
        merge_log_records,
    };

    /// Records straight out of a block whose CRC checked out.
    fn good(records: &[LogRecord]) -> Vec<ParsedLogRecord> {
        records.iter().cloned().map(ParsedLogRecord::good).collect()
    }

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
            air_brakes: AirBrakesActuationRecord {
                commanded_extension: Some(0.25),
                actual_extension: Some(0.2),
                validation_deploy: false,
            },
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
            launch_pad_altitude_asl: Some(200.0),
            air_brakes: AirBrakesRecord {
                predicted_apogee_asl: Some(3210.0),
                servo_temp: Some(41.5),
                target_apogee_asl: Some(3200.0),
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
        let parsed = parse_log_records(record_count, &wire[HEADER_LEN..], block_count).unwrap();
        assert_eq!(parsed.crc_failed_blocks, 0);
        assert_eq!(parsed.invalid_records, 0);
        let recovered: Vec<LogRecord> =
            parsed.records.iter().map(|r| r.record.clone()).collect();
        assert_eq!(recovered, log);
        assert!(parsed.records.iter().all(|r| r.block_crc_ok));

        let merged = merge_log_records(&parsed.records);
        assert_eq!(merged.len(), 20);
        assert_eq!(merged[0].record_count, 0);
        assert!(!merged[0].source_block_crc_failed);
        // The snapshot's own clock rides along, so a reader can bound how stale
        // the slow columns on this row are.
        assert_eq!(merged[0].slow_timestamp_us, Some(0));
        assert_eq!(merged[19].slow_timestamp_us, Some(15_000_000));
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
        let merged = merge_log_records(&good(&[LogRecord::Fast(sample_fast(0))]));
        assert_eq!(merged.len(), 1);
        assert!(merged[0].temperature.is_none());
        assert!(merged[0].num_of_fix_satellites.is_none());
        assert!(merged[0].air_brakes_mpc.is_none());
        // But the command and Icarus's report are fast-record fields, so they
        // are on the row regardless of whether a snapshot has landed yet.
        assert_eq!(merged[0].air_brakes.commanded_extension, Some(0.25));
        assert_eq!(merged[0].air_brakes.actual_extension, Some(0.2));
        assert!(merged[0].payload.is_none());
        assert!(merged[0].amp.is_none());
        assert!(merged[0].slow_timestamp_us.is_none());
        // Fast-record columns are unaffected.
        assert_eq!(merged[0].pressure, 101325.0);

        let merged = merge_log_records(&good(&[
            LogRecord::Slow(sample_slow(0)),
            LogRecord::Fast(sample_fast(0)),
        ]));
        assert_eq!(merged[0].temperature, Some(21.5));
        assert_eq!(merged[0].num_of_fix_satellites, Some(9));
        assert_eq!(merged[0].payload.as_ref().unwrap().rail_ma[2], None);
    }

    /// A session boundary is the same situation as
    /// [`merge_before_first_slow_record_reports_nothing`], except a stale
    /// snapshot is sitting there ready to be borrowed.
    ///
    /// One stored log spans several armed sessions and power cycles, because
    /// the logger resumes the existing log instead of starting a new one, and
    /// `sequence` stepping backwards is the only mark of where one ends. Left
    /// unhandled, every session after the first opened with ~42 rows carrying
    /// the *previous* session's GPS fix, node heartbeats, AMP and payload —
    /// data that looks exactly like a live reading.
    #[test]
    fn merge_drops_the_slow_snapshot_at_a_session_boundary() {
        let stale = FlightDataSlowRecord {
            temperature: 40.0,
            ..sample_slow(0)
        };
        let fresh = FlightDataSlowRecord {
            temperature: 21.5,
            ..sample_slow(9)
        };
        let merged = merge_log_records(&good(&[
            LogRecord::Slow(stale),
            LogRecord::Fast(sample_fast(5)),
            // `sequence` restarts: session two begins here.
            LogRecord::Fast(sample_fast(0)),
            LogRecord::Fast(sample_fast(1)),
            LogRecord::Slow(fresh),
            LogRecord::Fast(sample_fast(2)),
        ]));
        assert_eq!(merged.len(), 4);

        // Session one reads its own snapshot, as before.
        assert_eq!(merged[0].temperature, Some(40.0));
        assert_eq!(merged[0].slow_timestamp_us, Some(0));

        // Session two, before its first snapshot: nothing at all, exactly as if
        // the log had begun here. Not 40.0 C from a session that is over.
        for row in &merged[1..3] {
            assert!(row.temperature.is_none());
            assert!(row.lat_lon.is_none());
            assert!(row.gps_altitude_asl.is_none());
            assert!(row.num_of_fix_satellites.is_none());
            assert!(row.amp.is_none());
            assert!(row.amp_node.is_none());
            assert!(row.payload.is_none());
            assert!(row.air_brakes_mpc.is_none());
            assert!(row.slow_timestamp_us.is_none());
        }
        // Fast-record columns are untouched by the reset.
        assert_eq!(merged[1].record_count, 0);
        assert_eq!(merged[1].pressure, 101325.0);

        // ...and the slow columns come back once session two takes a snapshot.
        assert_eq!(merged[3].temperature, Some(21.5));
        assert_eq!(merged[3].slow_timestamp_us, Some(9_000_000));
    }

    /// A record from a CRC-failed block is exported, but marked — and the mark
    /// follows the snapshot, not just the record it arrived in.
    #[test]
    fn crc_failure_marks_every_row_it_touches() {
        let merged = merge_log_records(&[
            ParsedLogRecord {
                record: LogRecord::Slow(sample_slow(0)),
                block_crc_ok: false,
            },
            // A good fast record — but it carries the suspect snapshot, so the
            // row it produces is suspect too.
            ParsedLogRecord::good(LogRecord::Fast(sample_fast(0))),
            ParsedLogRecord {
                record: LogRecord::Fast(sample_fast(1)),
                block_crc_ok: false,
            },
        ]);
        assert_eq!(merged.len(), 2);
        assert!(merged[0].source_block_crc_failed);
        assert!(merged[1].source_block_crc_failed);
        // The data is still there to look at.
        assert_eq!(merged[0].temperature, Some(21.5));

        // A later good snapshot clears the taint for rows that only read it.
        let merged = merge_log_records(&[
            ParsedLogRecord {
                record: LogRecord::Slow(sample_slow(0)),
                block_crc_ok: false,
            },
            ParsedLogRecord::good(LogRecord::Slow(sample_slow(1))),
            ParsedLogRecord::good(LogRecord::Fast(sample_fast(0))),
        ]);
        assert!(!merged[0].source_block_crc_failed);
    }

    /// The important half of the CRC-failure fix: a corrupt discriminant must
    /// be rejected, not fed to a `match`.
    ///
    /// Every `Option` in these records archives to a `#[repr(u8)]` tag valid
    /// only as 0 or 1, and `FlightStage` / `NodeHealth` / `NodeMode` / `bool`
    /// are just as narrow. rkyv's unchecked deserialize would read a corrupted
    /// one straight into the generated `Deserialize` impl's `match`, which is
    /// undefined behaviour — reachable from a condition (a failed block CRC)
    /// the exporter already detects. Records are fixed width, so a corrupt body
    /// byte does not break parsing; it just quietly becomes an invalid value,
    /// which makes this the *typical* result of a CRC failure rather than a
    /// corner case.
    #[test]
    fn a_corrupt_body_is_rejected_rather_than_deserialized() {
        for (tag, wire_len) in [
            (RECORD_TAG_FAST, FAST_WIRE_LEN),
            (RECORD_TAG_SLOW, SLOW_WIRE_LEN),
        ] {
            let mut garbage = [0xAAu8; MAX_WIRE_LEN];
            garbage[0] = tag;
            assert!(
                deserialize_log_record_at(&garbage[..wire_len], 0).is_none(),
                "a body of 0xAA bytes decoded as a valid record"
            );
            // The width is still recoverable from the tag, which is what lets
            // the parser skip the bad record and keep going.
            assert_eq!(log_record_wire_len(&garbage[..wire_len]), Some(wire_len));
        }
    }

    /// The whole-log path: a bad block does not abort the export, its records
    /// are marked, and any record that fails validation is skipped instead of
    /// deserialised.
    #[test]
    fn a_crc_failed_block_still_exports_and_is_marked() {
        let log: Vec<LogRecord> = vec![
            LogRecord::Slow(sample_slow(0)),
            LogRecord::Fast(sample_fast(0)),
            LogRecord::Fast(sample_fast(1)),
        ];
        let n = log.len() as u32;
        let (mut blocks, _) = pack_log(&log);

        // Corrupt a byte inside the first record's body of every block and
        // leave the CRC trailers alone, which is what a bit-rotted card looks
        // like. Byte 3 is never a record tag, so the stream still walks.
        for block in blocks.iter_mut() {
            block[3] ^= 0xFF;
        }

        let mut wire = Vec::new();
        for b in &blocks {
            wire.extend_from_slice(b);
        }
        let parsed = parse_log_records(n, &wire, blocks.len() as u32).expect("still parses");
        assert_eq!(parsed.crc_failed_blocks, blocks.len() as u32);
        // Whatever survived validation is exported, and marked.
        assert!(parsed.records.iter().all(|r| !r.block_crc_ok));
        assert_eq!(
            parsed.records.len() as u32 + parsed.invalid_records,
            n,
            "every record is either returned or counted as skipped"
        );
        for row in merge_log_records(&parsed.records) {
            assert!(row.source_block_crc_failed);
        }
    }
}
