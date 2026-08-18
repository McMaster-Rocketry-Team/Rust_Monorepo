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
    /// born (`AIRBRAKES_BARO_TRUSTED`).
    pub kf_vertical_velocity: Option<f32>,
    /// Tilt from vertical (deg). `None` before ignition.
    pub kf_tilt_deg: Option<f32>,
    /// Status bits (`AIRBRAKES_*` consts): drag check, burnout latch,
    /// filter-born, apogee, and what its baro innovation gate did with THIS
    /// sample.
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
}

/// Airbrakes actuation: what was commanded, why, and what Icarus did with it.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct AirBrakesRecord {
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended. `None`
    /// until the firmware has commanded anything (i.e. outside Armed and Demo).
    ///
    /// Slow-rate because the control loop only produces one every 100 ms;
    /// logging it per fast record stored the same value ~42 times over.
    pub commanded_extension: Option<f32>,
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
    /// The commanded extension is the forced validation deploy, not the MPC's
    /// output: the MPC never asked for full extension the whole way up, so the
    /// firmware opened the brakes anyway once slow enough for it to be
    /// harmless, to leave in-flight evidence they actuate. While this is set,
    /// `commanded_extension` is 1.0 and `predicted_apogee_asl` is `None` —
    /// read the commanded column there as a servo test, not as MPC intent.
    pub validation_deploy: bool,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
    /// `None` until Icarus reports — which is the interesting case, since an
    /// Icarus that is offline or silent would otherwise be indistinguishable
    /// from one reporting fully-stowed brakes.
    ///
    /// Slow-rate because Icarus reports at 10 Hz; the reading is up to 100 ms
    /// older than this record's timestamp, so a commanded/actual pair on one
    /// row is not a step response.
    pub actual_extension: Option<f32>,
    /// Airbrakes servo temperature (C) reported by Icarus. `None` until Icarus
    /// reports, for the same reason as `actual_extension`.
    pub servo_temp: Option<f32>,
    /// Apogee ASL (m) the MPC is actually aiming at — the operator's AGL
    /// target plus the pad altitude, as the MPC latched the pair when it was
    /// constructed.
    ///
    /// Logged even though it is constant, because without it
    /// `predicted_apogee_asl` and `commanded_extension` cannot be read: a
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
    /// `predicted_apogee_asl` and a non-default `commanded_extension` are
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
    /// [`AirBrakesRecord::actual_extension`] and
    /// [`AirBrakesRecord::servo_temp`] come off Icarus at 10 Hz, so they can be
    /// a further ~100 ms older than this timestamp, and that latency is
    /// upstream of the snapshot where nothing here can see it.
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

    /// Airbrakes actuation from the slow snapshot (the control loop runs at
    /// 10 Hz, so there is nothing faster to log).
    pub air_brakes: Option<AirBrakesRecord>,

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
            air_brakes: slow.map(|s| s.air_brakes.clone()),
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
/// The mach-lockout exit is a single drag measurement (the drag-inverted
/// airspeed below Mach 0.8, sustained 1 s), so there is one bit for it;
/// logging it per sample reconstructs the exit post-flight.
pub const AIRBRAKES_SUBSONIC_DRAG: u8 = 1 << 0;
/// The vertical filter's innovation gate threw out this sample's baro
/// altitude. Per-sample, like the `DEPLOYMENT_*` pair above, so a run of set
/// bits is one rejection episode — an ejection transient, or a port the shock
/// front has disturbed. Only ever set while the vertical filter exists
/// (`AIRBRAKES_BARO_TRUSTED`).
pub const AIRBRAKES_BARO_GATE_REJECT: u8 = 1 << 2;
/// The axial-sign burnout latch has fired: the motor is out and the drag
/// channel is honest. Nothing can birth the vertical filter before this, on
/// either the supersonic or the subsonic path, so it separates "the brakes
/// never opened because the motor never looked out" from "because the drag
/// check never passed".
pub const AIRBRAKES_BURNOUT: u8 = 1 << 1;
/// The vertical filter is born (baro trusted; MPC state is live).
pub const AIRBRAKES_BARO_TRUSTED: u8 = 1 << 3;
/// This sample ended a rejection run by re-anchoring: altitude snapped to the
/// baro and velocity uncertainty was re-opened. Set together with
/// `AIRBRAKES_BARO_GATE_REJECT`. A run that ends without this bit is the gate
/// doing its job; a run that ends with it is a diverged filter.
pub const AIRBRAKES_BARO_RESYNC: u8 = 1 << 4;
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
pub const AIRBRAKES_PAD_CALIBRATED: u8 = 1 << 5;
/// The brakes were permitted to open on this sample: the airbrakes filter is
/// alive, its barometer is trusted, and its own velocity is below
/// `max_open_mach` of the local speed of sound. This is the MPC's run/stop
/// condition, logged per sample.
///
/// The only bit here that is a decision rather than an observation, and it is
/// stored rather than re-derived because it cannot be re-derived: the Mach
/// term compares against a config constant and a speed of sound computed from
/// the filter's own altitude, neither of which is in the log. The first row it
/// is set on is the instant the control loop was allowed to start; a run of
/// clear rows in the middle of the coast is the gate having closed again.
pub const AIRBRAKES_MPC_PERMITTED: u8 = 1 << 6;
// bit 7 unallocated.

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
