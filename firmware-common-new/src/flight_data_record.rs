use crate::can_bus::messages::{
    node_status::{NodeHealth, NodeMode, NodeStatusMessage},
    vl_status::FlightStage,
};

/// One CAN node's last `NodeStatusMessage`, stored whole.
///
/// The downlink packet compresses each node to two bits (online + a
/// `uptime_s < 5` reboot flag) because it has no room for more; the SD log
/// has room, so it keeps everything. That is what makes "the node went
/// unhealthy at T+12 s" or "it rebooted mid-flight" answerable after the
/// fact — a reboot shows as `uptime_s` stepping backwards, which the packet's
/// derived bit can miss entirely if it happens between two 2 s packets.
///
/// A node never heard from at all has no record: the slot in the slow record
/// is `None`. Every `NodeStatusRecord` that exists therefore describes a
/// heartbeat that really arrived.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct NodeStatusRecord {
    /// Nothing heard from the node for 5 s. When false, every other field
    /// describes the last heartbeat received rather than the present.
    pub online: bool,
    /// Seconds since the node booted. A step backwards is a reboot.
    pub uptime_s: u32,
    pub health: NodeHealth,
    pub mode: NodeMode,
    /// Node-specific flags, 11 bits. Decode with that node's custom-status
    /// type: `OzysCustomStatus`, `PayloadSDRMCustomStatus`, `VLCustomStatus`.
    pub custom_status: u16,
}

impl NodeStatusRecord {
    pub fn from_message(online: bool, message: &NodeStatusMessage) -> Self {
        Self {
            online,
            uptime_s: message.uptime_s,
            health: message.health,
            mode: message.mode,
            custom_status: message.custom_status_raw,
        }
    }
}

/// High-rate IMU / baro / mag / estimator / pyro sample.
pub const RECORD_TAG_FAST: u8 = 0x01;
/// Low-rate GPS / battery / AMP / temperature / airbrakes-actuation snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

/// One IMU sample: both halves come from the same read, so they are present
/// or absent together.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct ImuRecord {
    pub acc: [f32; 3],
    pub gyro: [f32; 3],
}

/// The deployment (slow baro) estimator's output for one fast sample.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct DeploymentEstimatorRecord {
    /// Estimator altitude ASL (m). `None` through the Mach lockout, where the
    /// KF is frozen and nothing is fused at all.
    pub kf_altitude_asl: Option<f32>,
    /// Estimator vertical velocity (m/s). `None` for the same reason as
    /// `kf_altitude_asl` — the two are frozen and released together.
    pub kf_vertical_velocity: Option<f32>,
    /// Status bits (`DEPLOYMENT_*` consts): what the baro innovation gate did
    /// with THIS sample.
    pub flags: u8,
}

/// The airbrakes estimator's output for one fast sample.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct AirbrakesEstimatorRecord {
    /// Estimator altitude ASL (m). `None` until it has a value.
    pub kf_altitude_asl: Option<f32>,
    /// Estimator vertical velocity (m/s). `None` until its vertical filter is
    /// born ([`AirbrakesState::AirbrakesEnabled`]).
    pub kf_vertical_velocity: Option<f32>,
    /// Tilt from vertical (deg). `None` before ignition.
    pub kf_tilt_deg: Option<f32>,
    /// Status bits (`AIRBRAKES_*` consts): the two-bit [`AirbrakesState`],
    /// the drag check, the burnout latch and the pad calibration.
    pub flags: u8,
}

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataFastRecord {
    /// Per-record counter, restarting at 0 each armed session — the logger runs
    /// inside armed mode, so a re-arm (or a reboot) begins a new count.
    ///
    /// Within a session, a step of anything other than +1 means samples were
    /// lost anywhere between the sensor and the SD card — the logger fell
    /// behind the sensor stream, the SD queue was full, or the card was
    /// offline. Each of those consumes its sequence number, so a gap here is
    /// the one place every kind of loss shows up. A step *backwards* is a
    /// session boundary, not a drop: one stored log can hold several armed
    /// sessions (nothing is logged in between), and this is what marks where
    /// one ends.
    pub sequence: u32,
    pub timestamp_us: u64,
    /// GPS-disciplined unix clock, microseconds since the epoch. `None` until
    /// the clock is ready (no time fix yet).
    pub unix_time_us: Option<u64>,
    /// `None` when no IMU sample backed this tick — the driver was riding out
    /// a bus error or had not produced a reading yet.
    pub imu: Option<ImuRecord>,
    /// Barometric pressure (Pa). Every published sample carries one, so this
    /// is never absent: the fast records are driven by the baro stream.
    pub pressure: f32,
    /// Magnetometer field (µT). `None` when no mag sample backed this tick,
    /// same causes as `imu`.
    pub mag: Option<[f32; 3]>,
    /// `None` when no deployment estimator sample matched this tick.
    pub deployment: Option<DeploymentEstimatorRecord>,
    /// `None` when the airbrakes estimator produced nothing for this tick:
    /// it is not born yet, or it was retired at apogee.
    pub airbrakes: Option<AirbrakesEstimatorRecord>,
    /// Mirror of the deployment estimator's `RocketState` (plus the device
    /// modes), with its Mach lockout folded into `Ascent`. Logged only here,
    /// at the full fast rate — the chutes' deployment shows up as this
    /// changing stage, and the pyro edges themselves as `pyro_flags`.
    pub flight_stage: FlightStage,
    /// Bitmask for pyro continuity/fire state (`PYRO_*` consts). Logged at
    /// the full fast rate so pyro fire edges are timestamped to ±2.3 ms.
    /// `None` until there is anything on the continuity watch to report.
    pub pyro_flags: Option<u8>,
    /// Brake command and Icarus's reported extension, at the full fast rate
    /// for the same reason as `pyro_flags`: the edges are the measurement.
    ///
    /// Not behind an `Option` like the other groups, because it is not one
    /// source that comes and goes — the command and the report arrive from
    /// different places at different times, and each carries its own absence.
    pub air_brakes: AirBrakesActuationRecord,
}

/// Airbrakes actuation: what was commanded, why, and what Icarus reports back.
///
/// Fast-rate, and the only group here that is about *latency*. Neither number
/// changes faster than 100 Hz — the control loop commands at 10 Hz, Icarus
/// measures and reports at 100 Hz — so this is not stored at 427 Hz to resolve
/// the values themselves. It is stored at 427 Hz to timestamp their EDGES to
/// ±2.3 ms, which is what makes a commanded step and the actual extension that
/// follows it a measurable step response instead of two columns quantised onto
/// a shared 100 ms grid. On the slow record each edge cost ~100 ms of
/// quantisation plus up to another 100 ms of snapshot staleness — the same
/// order as the servo travel being measured.
///
/// Both are `None` until their source has spoken — the firmware for the
/// command, Icarus for the report — so an offline Icarus reads as an empty
/// cell, never as stowed brakes.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct AirBrakesActuationRecord {
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended. `None`
    /// until the firmware has commanded anything (i.e. outside Armed and Demo).
    ///
    /// The control loop only produces one every 100 ms, so the value repeats
    /// across ~42 rows; the row it first changes on is the point of logging it
    /// here.
    pub commanded_extension: Option<f32>,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
    /// `None` until Icarus reports — which is the interesting case, since an
    /// Icarus that is offline or silent would otherwise be indistinguishable
    /// from one reporting fully-stowed brakes.
    ///
    /// Icarus measures the servo angle every cycle of its 100 Hz control loop
    /// and now reports every one of them, so the reading is at most one Icarus
    /// cycle (~10 ms) plus one CAN hop older than this record's timestamp.
    pub actual_extension: Option<f32>,
    /// The commanded extension is the forced validation deploy, not the MPC's
    /// output: the MPC never asked for full extension the whole way up, so the
    /// firmware opened the brakes anyway once slow enough for it to be
    /// harmless, to leave in-flight evidence they actuate. While this is set,
    /// `commanded_extension` is 1.0 and [`AirBrakesRecord::predicted_apogee_asl`]
    /// is `None` — read the commanded column there as a servo test, not as MPC
    /// intent.
    ///
    /// Here rather than beside the prediction it also qualifies, because what
    /// it qualifies first is the command: the two must not be able to disagree
    /// about which row the validation deploy started on.
    pub validation_deploy: bool,
}

/// The airbrakes numbers that genuinely only move at 10 Hz: what the MPC
/// predicts and aims at, and how hot the servo is.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct AirBrakesRecord {
    /// Apogee ASL (m) the MPC predicts at the extension it is commanding.
    /// `None` whenever the MPC is not running: before the brakes are
    /// permitted, again once the airbrakes estimator is retired and it stops,
    /// and throughout the validation deploy.
    ///
    /// ASL, like every other altitude in this log, and like the MPC's own
    /// internal number — the AGL the downlink carries is this minus
    /// [`FlightDataSlowRecord::launch_pad_altitude_asl`], which is on the
    /// same row, so the conversion is available offline in a way it was not
    /// when the log stored the difference and not the reference.
    pub predicted_apogee_asl: Option<f32>,
    /// Airbrakes servo temperature (C) reported by Icarus. `None` until Icarus
    /// reports, for the same reason as
    /// [`AirBrakesActuationRecord::actual_extension`].
    ///
    /// Slow-rate for the reason the extension beside it is not: Icarus reads
    /// the servo's temperature once per ten control cycles and repeats it in
    /// the reports between, so there is nothing here a 10 Hz row cannot hold.
    pub servo_temp: Option<f32>,
    /// Apogee ASL (m) the MPC is actually aiming at — the operator's AGL
    /// target plus the pad altitude, as the MPC latched the pair when it was
    /// constructed.
    ///
    /// Logged even though it is constant, because without it
    /// `predicted_apogee_asl` and the commanded extension cannot be read: a
    /// prediction well above target with the brakes barely open is a broken
    /// controller if the target was reachable and correct behaviour if it was
    /// not, and the log alone could not tell those apart. A bench flight on
    /// 2026-08-17 hit exactly that — a stale 9448 m target above the 9348 m
    /// natural apogee, which is why the MPC never saturated and the validation
    /// deploy fired.
    ///
    /// This is the MPC's own value rather than a per-record sample of the
    /// operator's target watch, and the two are not the same claim: the MPC
    /// takes its target once, at construction, so a `SetTargetApogee` accepted
    /// later in the flight moves the watch and the SD config block but not the
    /// number the controller is chasing. The log now says what the controller
    /// is doing; the operator's latest setting is on the card either way, in
    /// the config block.
    ///
    /// `None` until the MPC is constructed, which is the same window
    /// `predicted_apogee_asl` and a non-default commanded extension are
    /// absent for — before it there is no target, only a setting.
    pub target_apogee_asl: Option<f32>,
}

/// Last `AmpStatusMessage`, which is a different stream from the AMP node
/// heartbeat — this is the power board's own report, so it is present or
/// absent independently of `amp_node`.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct AmpRecord {
    /// Shared (AMP) battery voltage.
    pub shared_battery_v: f32,
    /// AMP output statuses, 2 bits per output with out1 in the LSBs. Each
    /// pair holds a `PowerOutputStatus` discriminant.
    pub out_status: u8,
}

/// Last `CustomPayloadStatusMessage`, in the units that message carries.
///
/// Each reading is separately `None`: the payload marks individual readings
/// unavailable, so a live EPM with one dead rail sensor is a real state and
/// stays distinguishable from a payload that has said nothing at all (every
/// field `None`).
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct PayloadRecord {
    /// Payload EPM battery bus voltage (mV).
    pub epm_batt_mv: Option<u16>,
    /// Payload EPM switched rail load currents (mA), rail index order 0 `SYS_3V3`,
    /// 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`, 4 `PER_9V`, 5 `PER_12V`.
    pub rail_ma: [Option<u16>; 6],
    /// SEM linear actuator positions (steps), experiment channels 1..3.
    pub actuator_steps: [Option<u16>; 3],
}

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataSlowRecord {
    pub timestamp_us: u64,
    /// MS5607 die temperature (C). Slow-rate because the driver only sources
    /// it once per `TEMP_DECIMATION` pressure reads (~13 Hz) — logging it per
    /// fast record stored the same value ~30 times over. Sourced from the same
    /// stream that drives the fast records, so it is always present.
    pub temperature: f32,
    /// VL battery voltage. `None` until the ADC has produced a reading.
    pub battery_voltage: Option<f32>,
    /// `None` until the GPS has a position fix.
    pub lat_lon: Option<(f64, f64)>,
    /// GPS-reported altitude, metres above mean sea level (≈ASL). `None` until
    /// the fix carries an altitude — it can be absent while `lat_lon` is not.
    pub gps_altitude_asl: Option<f32>,
    /// Satellites used in the fix. 0 is a real reading (no fix), so it is not
    /// optional.
    pub num_of_fix_satellites: u8,
    /// Dilution of precision, `None` when the GPS did not report it.
    pub hdop: Option<f32>,
    pub vdop: Option<f32>,
    pub pdop: Option<f32>,
    /// Launch pad altitude (m ASL) held by the deployment estimator — the ONE
    /// reference every AGL number in the firmware is measured from: the pyro
    /// thresholds, the downlink's altitude and apogee, and the MPC's target.
    ///
    /// This is what makes the log self-contained. Every altitude stored here
    /// is ASL, which is the honest unit — it is what the barometer and the GPS
    /// both measure, and it needs no reference to interpret — but the numbers
    /// the flight actually acted on are AGL, and until this field existed the
    /// log had no way to reproduce them. The pad reference lived only inside
    /// the estimator and in the downlink's already-subtracted altitudes, so a
    /// finished log could not answer "what did the firmware think its AGL
    /// was?" at all.
    ///
    /// A low-passed barometer reading while the rocket is on the rail, and a
    /// constant latched at ignition detection afterwards, so it moves only
    /// during the pad segment and is flat for the whole flight.
    ///
    /// `None` only when this record's tick had no matching estimator sample —
    /// the same condition that empties the fast record's `deployment` group,
    /// i.e. before the estimator's first sample of the session.
    pub launch_pad_altitude_asl: Option<f32>,
    pub air_brakes: AirBrakesRecord,
    /// `None` until the first `AmpStatusMessage` heartbeat arrives.
    pub amp: Option<AmpRecord>,
    pub payload: PayloadRecord,
    /// Full `NodeStatusMessage` for each node on the bus, as last received.
    /// `None` means never heard from at all — as opposed to a record with
    /// `online: false`, which is a node that spoke and then went quiet.
    pub amp_node: Option<NodeStatusRecord>,
    pub icarus_node: Option<NodeStatusRecord>,
    pub ozys_node: Option<NodeStatusRecord>,
    pub payload_sdrm_node: Option<NodeStatusRecord>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LogRecord {
    Fast(FlightDataFastRecord),
    Slow(FlightDataSlowRecord),
}

/// One record as read back off the card, tagged with the health of the
/// 512-byte block it was found in.
///
/// The block CRC is checked per block, but a block holds ~4-5 records, so the
/// question a reader actually asks — "can I trust THIS row?" — is only
/// answerable if the answer travels with the record. That is what this carries.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedLogRecord {
    pub record: LogRecord,
    /// `false` when the block this record came out of failed its CRC32
    /// trailer: some byte in that block is wrong, and it may well be one of
    /// this record's.
    pub block_crc_ok: bool,
}

impl ParsedLogRecord {
    /// A record from a block whose CRC checked out.
    pub fn good(record: LogRecord) -> Self {
        Self {
            record,
            block_crc_ok: true,
        }
    }
}

/// Merged view used for CSV export (one row per fast record).
///
/// Everything sourced from the slow snapshot is `Option` here, including the
/// fields the slow record itself always carries: rows before the first slow
/// record have no snapshot to read from. A reader draws the same conclusion
/// either way — nothing has been reported for this column yet — so those
/// fields are a single `Option`, not a nested one.
#[derive(Debug, Clone, PartialEq)]
pub struct FlightDataRecord {
    /// [`FlightDataFastRecord::sequence`] — see there for what a discontinuity
    /// means (a drop forwards, a session boundary backwards).
    pub record_count: u32,
    /// The 512-byte block behind some of this row's data failed its CRC32
    /// trailer, so at least one byte in it is wrong. Set when either the fast
    /// sample or the slow snapshot the row carries came out of a bad block.
    ///
    /// Such rows are still exported — one bad block must not make an otherwise
    /// good flight log unrecoverable — but nothing on them is trustworthy.
    /// Records are fixed width, so a corrupt body byte cannot desynchronize the
    /// stream: it silently changes a value here instead of breaking parsing,
    /// which is exactly why the row has to be marked rather than left to look
    /// like every other row.
    pub source_block_crc_failed: bool,
    pub timestamp_us: u64,
    /// [`FlightDataSlowRecord::timestamp_us`] of the snapshot the slow columns
    /// on this row were copied from, `None` when no snapshot precedes the row.
    ///
    /// `timestamp_us - slow_timestamp_us` bounds how stale the VL-side snapshot
    /// is (at most one slow period, ~100 ms). That bound is about the SNAPSHOT
    /// ONLY. It says nothing about how old the readings inside it are:
    /// [`AirBrakesRecord::servo_temp`] comes off Icarus at 10 Hz, so it can be
    /// a further ~100 ms older than this timestamp, and that latency is
    /// upstream of the snapshot where nothing here can see it. The extension
    /// pair used to be the example here; it is on the fast record now, which
    /// is why it no longer is.
    pub slow_timestamp_us: Option<u64>,
    /// GPS-disciplined unix clock (µs since epoch), `None` until the clock is
    /// ready.
    pub unix_time_us: Option<u64>,

    pub imu: Option<ImuRecord>,

    pub pressure: f32,

    pub mag: Option<[f32; 3]>,

    pub deployment: Option<DeploymentEstimatorRecord>,

    pub airbrakes: Option<AirbrakesEstimatorRecord>,

    /// MS5607 die temperature (C), from the slow snapshot.
    pub temperature: Option<f32>,
    pub battery_voltage: Option<f32>,

    pub lat_lon: Option<(f64, f64)>,
    /// GPS-reported altitude, metres above mean sea level (≈ASL).
    pub gps_altitude_asl: Option<f32>,
    pub num_of_fix_satellites: Option<u8>,
    pub hdop: Option<f32>,
    pub vdop: Option<f32>,
    pub pdop: Option<f32>,
    /// Launch pad altitude (m ASL) from the slow snapshot — the reference every
    /// AGL number on this row is measured from. See
    /// [`FlightDataSlowRecord::launch_pad_altitude_asl`].
    pub launch_pad_altitude_asl: Option<f32>,

    /// From the fast record, so it is full-rate.
    pub flight_stage: FlightStage,

    /// Bitmask for pyro continuity/fire state (see firmware `ContinuityUpdate`).
    /// Full rate, from the fast record.
    pub pyro_flags: Option<u8>,

    /// Brake command and Icarus's report, from the fast record — so a
    /// commanded step and the extension that follows it are on rows 2.3 ms
    /// apart, not on the same 100 ms grid point.
    pub air_brakes: AirBrakesActuationRecord,

    /// The MPC's prediction and target, and the servo temperature, from the
    /// slow snapshot.
    pub air_brakes_mpc: Option<AirBrakesRecord>,

    /// CAN node heartbeats from the slow record.
    pub amp_node: Option<NodeStatusRecord>,
    pub icarus_node: Option<NodeStatusRecord>,
    pub ozys_node: Option<NodeStatusRecord>,
    pub payload_sdrm_node: Option<NodeStatusRecord>,
    /// AMP power-board report from the slow record.
    pub amp: Option<AmpRecord>,

    /// Payload snapshot from the slow record, in the units the payload CAN
    /// message carries.
    pub payload: Option<PayloadRecord>,
}

impl FlightDataRecord {
    /// Combine one fast sample with the most recent slow snapshot, or `None`
    /// if no slow record has been seen yet.
    ///
    /// `source_block_crc_failed` covers the whole row: pass `true` if either
    /// the fast record or the snapshot came from a block that failed its CRC.
    pub fn from_fast_and_slow(
        fast: &FlightDataFastRecord,
        slow: Option<&FlightDataSlowRecord>,
        source_block_crc_failed: bool,
    ) -> Self {
        Self {
            record_count: fast.sequence,
            source_block_crc_failed,
            timestamp_us: fast.timestamp_us,
            slow_timestamp_us: slow.map(|s| s.timestamp_us),
            unix_time_us: fast.unix_time_us,
            imu: fast.imu.clone(),
            pressure: fast.pressure,
            mag: fast.mag,
            deployment: fast.deployment.clone(),
            airbrakes: fast.airbrakes.clone(),
            temperature: slow.map(|s| s.temperature),
            battery_voltage: slow.and_then(|s| s.battery_voltage),
            lat_lon: slow.and_then(|s| s.lat_lon),
            gps_altitude_asl: slow.and_then(|s| s.gps_altitude_asl),
            num_of_fix_satellites: slow.map(|s| s.num_of_fix_satellites),
            hdop: slow.and_then(|s| s.hdop),
            vdop: slow.and_then(|s| s.vdop),
            pdop: slow.and_then(|s| s.pdop),
            launch_pad_altitude_asl: slow.and_then(|s| s.launch_pad_altitude_asl),
            flight_stage: fast.flight_stage,
            pyro_flags: fast.pyro_flags,
            air_brakes: fast.air_brakes.clone(),
            air_brakes_mpc: slow.map(|s| s.air_brakes.clone()),
            amp_node: slow.and_then(|s| s.amp_node.clone()),
            icarus_node: slow.and_then(|s| s.icarus_node.clone()),
            ozys_node: slow.and_then(|s| s.ozys_node.clone()),
            payload_sdrm_node: slow.and_then(|s| s.payload_sdrm_node.clone()),
            amp: slow.and_then(|s| s.amp.clone()),
            payload: slow.map(|s| s.payload.clone()),
        }
    }
}

/// Expand a tagged log into merged rows (one CSV row per fast sample).
///
/// The held slow snapshot is dropped at every session boundary. One stored log
/// spans several armed sessions and several power cycles, because the firmware
/// logger resumes an existing log rather than starting a new one, and
/// [`FlightDataFastRecord::sequence`] stepping backwards is the only marker of
/// where one session ends. Carrying a snapshot across it would put the previous
/// session's GPS fix, node heartbeats, AMP and payload state on the first ~42
/// rows of the next one, which is the same lie as inventing slow data before
/// the first snapshot ever arrives.
///
/// The reset is deliberately blunt: a snapshot written after the boundary but
/// before the new session's first fast record is discarded with it. That costs
/// at most one slow period (~100 ms) of real data at each boundary — the
/// records interleave at ~42 fast per slow, so a session almost always opens on
/// a fast record — and it is the only direction that cannot mislabel a row.
#[cfg(any(feature = "std", test))]
pub fn merge_log_records(log: &[ParsedLogRecord]) -> std::vec::Vec<FlightDataRecord> {
    let mut slow: Option<FlightDataSlowRecord> = None;
    // Tracked alongside `slow` because a row is untrustworthy if EITHER half of
    // it came from a bad block, and the snapshot outlives the record it came in.
    let mut slow_from_bad_block = false;
    let mut prev_sequence: Option<u32> = None;
    let mut out = std::vec::Vec::new();
    for parsed in log {
        match &parsed.record {
            LogRecord::Slow(s) => {
                slow = Some(s.clone());
                slow_from_bad_block = !parsed.block_crc_ok;
            }
            LogRecord::Fast(fast) => {
                if prev_sequence.is_some_and(|prev| fast.sequence < prev) {
                    slow = None;
                    slow_from_bad_block = false;
                }
                prev_sequence = Some(fast.sequence);
                out.push(FlightDataRecord::from_fast_and_slow(
                    fast,
                    slow.as_ref(),
                    !parsed.block_crc_ok || slow_from_bad_block,
                ))
            }
        }
    }
    out
}

/// `DeploymentEstimatorRecord::flags` bits — the deployment estimator's status.
///
/// Both bits describe **this record's sample**, not a running state: they are
/// read in the same critical section as the estimator update that produced
/// them, so a single-sample event cannot be missed or land on the wrong row.
/// Both read 0 through Mach lockout, where the KF is frozen and nothing is
/// fused at all — which is also where the altitude and velocity are `None`.
///
/// The deployment KF is the one that fires pyros, so its gate is the first
/// thing to look at when a deploy went wrong.
pub const DEPLOYMENT_BARO_GATE_REJECT: u8 = 1 << 0;
/// This sample ended a rejection run by snapping altitude to the baro: the
/// filter, not the sensor, was judged wrong. Set together with
/// `DEPLOYMENT_BARO_GATE_REJECT`, and altitude is discontinuous across the
/// row it appears on.
pub const DEPLOYMENT_BARO_RESYNC: u8 = 1 << 1;
// bits 2-7 unallocated.

/// `AirbrakesEstimatorRecord::flags` bits — the airbrakes estimator's status.
///
/// Three flags and a two-bit state, packed from the bottom with bits 5-7
/// free. There is no reserved hole: bit 3 was one for a while, held by
/// `AIRBRAKES_BARO_TRUSTED` and then briefly `AIRBRAKES_ENABLED`, and bits 2
/// and 4 were the baro innovation gate's pair. Nothing decodes an older
/// layout — a card at any other `STORAGE_VERSION` is rejected outright — so a
/// hole in this byte protects nobody and only makes the next reader wonder
/// what used to be in it.
///
/// The mach-lockout drag check voted subsonic on this sample — the
/// drag-inverted airspeed was below `max_open_mach` — and the filter was
/// NOT born on it.
///
/// Normally 0 for an entire flight, which is the point. The check used to
/// need a continuous second before it could conclude, so this bit marked
/// that second and every flight had a run of it; since it concludes on the
/// sample it votes on, the vote and the birth are the same row and the birth
/// is already visible as the state going to `AirbrakesEnabled`. What is left
/// is the disagreement case: the check said go and something at the birth
/// site refused — the inertial Mach test on the dead reckoner's own
/// velocity, or a baro ring too empty to take a median from. A run of these
/// is the drag model reading the airframe faster than the accelerometers do,
/// which is the one thing about the lockout exit that nothing else in the
/// log would show.
pub const AIRBRAKES_SUBSONIC_DRAG: u8 = 1 << 0;
/// The axial-sign burnout latch has fired: the motor is out and the drag
/// channel is honest. Nothing can birth the vertical filter before this, on
/// either the supersonic or the subsonic path, so it separates "the brakes
/// never opened because the motor never looked out" from "because the drag
/// check never passed".
pub const AIRBRAKES_BURNOUT: u8 = 1 << 1;
/// The pad calibration completed: gyro bias, pad orientation and pad altitude
/// exist, and the estimator is willing to detect ignition.
///
/// Unlike every other bit here this one is about the PAD, and it is the only
/// row content the log carries before ignition. While it is clear the
/// airbrakes cannot fly at all — ignition detection is gated on it — so a log
/// that opens with a run of zeros here explains an otherwise silent
/// no-deployment. Expect it set within ~6 s of the estimator starting and to
/// stay set; it is re-derived every 2 s and CAN drop if the airframe is
/// picked up or turned.
pub const AIRBRAKES_PAD_CALIBRATED: u8 = 1 << 2;

/// The two bits of `AirbrakesEstimatorRecord::flags` holding
/// [`AirbrakesState`].
pub const AIRBRAKES_STATE_SHIFT: u32 = 3;
pub const AIRBRAKES_STATE_MASK: u8 = 0b0001_1000;

/// Which state of the airbrakes estimator produced this sample.
///
/// The estimator is a four-state machine that only ever walks forward, so two
/// bits hold it exactly and every question about "where was it" becomes a
/// comparison rather than an inference across three flags. It is logged
/// rather than derived because two of the four were not distinguishable from
/// the outside at all: `Armed` and `Stage1` differ only by the airbrakes
/// half's OWN ignition detection, which runs a separate detector from the one
/// that moves `flight_stage` and can latch a sample or two apart from it.
///
/// This replaced a bit that meant "the vertical filter exists". That bit was
/// true in exactly one state and so said the same thing as
/// `kf_altitude_asl.is_some()` — the last `*_valid` flag in the format,
/// arrived at from the other direction.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AirbrakesState {
    /// Armed on the pad: screening for a calibration, watching for ignition.
    /// No orientation, no altitude, no velocity.
    Armed = 0,
    /// The first half second after this half detected ignition, during which
    /// the accumulated thrust direction solves how the avionics are mounted.
    /// Still no tilt — the axis it is measured against is what this state is
    /// computing.
    Stage1 = 1,
    /// Boost and the Mach lockout: inertial dead reckoning, tilt available,
    /// the barometer buffered but never fused. The burnout latch and the drag
    /// check both live here, which is why `AIRBRAKES_BURNOUT` and
    /// `AIRBRAKES_SUBSONIC_DRAG` only ever say anything in this state.
    DeadReckoning = 2,
    /// The brakes may open, and will be allowed to for the rest of the
    /// flight: motor out, drag check (or the T_max backstop) passed, vertical
    /// filter alive and fusing the baro, and the airframe under
    /// `max_open_mach` when it got here. One-way like the rest — the only way
    /// out is the estimator being dropped whole at apogee, which shows up as
    /// the airbrakes group going absent.
    AirbrakesEnabled = 3,
}

impl AirbrakesState {
    pub fn from_flags(flags: u8) -> Self {
        match (flags & AIRBRAKES_STATE_MASK) >> AIRBRAKES_STATE_SHIFT {
            0 => Self::Armed,
            1 => Self::Stage1,
            2 => Self::DeadReckoning,
            // Two bits, four states, all four named: there is no unknown
            // variant to fall through to, and inventing one would be a
            // permanently dead branch.
            _ => Self::AirbrakesEnabled,
        }
    }

    pub fn to_flags(self) -> u8 {
        (self as u8) << AIRBRAKES_STATE_SHIFT
    }
}

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
