use crate::can_bus::messages::{
    custom_payload_status::PAYLOAD_READING_UNAVAILABLE,
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
    /// Never heard from, or silent for 5 s. Matches the downlink packet's
    /// `NodeStatus::offline()` so the two channels agree on what absence
    /// looks like.
    pub fn offline() -> Self {
        Self {
            online: false,
            uptime_s: 0,
            health: NodeHealth::Error,
            mode: NodeMode::Offline,
            custom_status: 0,
        }
    }

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

impl Default for NodeStatusRecord {
    fn default() -> Self {
        Self::offline()
    }
}

/// High-rate IMU / baro / mag / estimator / pyro sample.
pub const RECORD_TAG_FAST: u8 = 0x01;
/// Low-rate GPS / battery / AMP / temperature / airbrakes-actuation snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

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
    /// GPS-disciplined unix clock, microseconds since the epoch. 0 until the
    /// clock is ready (no time fix yet).
    pub unix_time_us: u64,
    pub acc: [f32; 3],
    pub gyro: [f32; 3],
    pub pressure: f32,
    pub mag: [f32; 3],
    /// Deployment (slow baro) estimator altitude ASL (m). NaN until the
    /// estimator has run.
    pub deployment_kf_altitude_asl: f32,
    /// Deployment (slow baro) estimator vertical velocity (m/s). NaN until
    /// the estimator has run.
    pub deployment_kf_vertical_velocity: f32,
    /// Deployment estimator status bits (`DEPLOYMENT_*` consts): what the
    /// baro innovation gate did with THIS sample.
    pub deployment_flags: u8,
    /// Airbrakes estimator altitude ASL (m). NaN until it has a value.
    pub airbrakes_kf_altitude_asl: f32,
    /// Airbrakes estimator vertical velocity (m/s). NaN until its vertical
    /// filter is born.
    pub airbrakes_kf_vertical_velocity: f32,
    /// Airbrakes estimator tilt from vertical (deg). NaN before ignition.
    pub airbrakes_kf_tilt_deg: f32,
    /// Airbrakes estimator status bits (`AIRBRAKES_*` consts): drag check,
    /// burnout latch, filter-born, apogee, and what its baro innovation gate
    /// did with THIS sample.
    pub airbrakes_flags: u8,
    /// Mirror of the deployment estimator's `RocketState` (plus the device
    /// modes), with its Mach lockout folded into `Ascent`. Logged only here,
    /// at the full fast rate — the chutes' deployment shows up as this
    /// changing stage, and the pyro edges themselves as `pyro_flags`.
    pub flight_stage: FlightStage,
    /// Bitmask for pyro continuity/fire state (`PYRO_*` consts). Logged at
    /// the full fast rate so pyro fire edges are timestamped to ±2.3 ms.
    pub pyro_flags: u8,
    pub valid: u8,
}

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataSlowRecord {
    pub timestamp_us: u64,
    /// MS5607 die temperature (C). Slow-rate because the driver only sources
    /// it once per `TEMP_DECIMATION` pressure reads (~13 Hz) — logging it per
    /// fast record stored the same value ~30 times over.
    pub temperature: f32,
    pub battery_voltage: f32,
    pub lat_lon: (f64, f64),
    /// GPS-reported altitude, metres above mean sea level (≈ASL).
    pub gps_altitude_asl: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended. NaN until
    /// the firmware has commanded anything (i.e. outside Armed and Demo).
    ///
    /// Slow-rate because the control loop only produces one every 100 ms;
    /// logging it per fast record stored the same value ~42 times over.
    pub air_brakes_commanded_extension: f32,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
    /// NaN until Icarus reports — which is the interesting case, since an
    /// Icarus that is offline or silent would otherwise be indistinguishable
    /// from one reporting fully-stowed brakes.
    ///
    /// Slow-rate because Icarus reports at 10 Hz; the reading is up to 100 ms
    /// older than this record's timestamp, so a commanded/actual pair on one
    /// row is not a step response.
    pub air_brakes_actual_extension: f32,
    /// Airbrakes servo temperature (C) reported by Icarus. NaN until Icarus
    /// reports, for the same reason as `air_brakes_actual_extension`.
    pub air_brakes_servo_temp: f32,
    /// The commanded extension is the forced validation deploy, not the MPC's
    /// output: the MPC never asked for full extension the whole way up, so the
    /// firmware opened the brakes anyway once slow enough for it to be
    /// harmless, to leave in-flight evidence they actuate. While this is set,
    /// `air_brakes_commanded_extension` is 1.0 and
    /// `mpc_predicted_apogee_agl` is NaN — read the commanded column there as
    /// a servo test, not as MPC intent.
    pub air_brakes_validation_deploy: bool,
    /// Apogee AGL (m) the MPC predicts at the extension it is commanding.
    /// NaN until the MPC runs (it starts only once the brakes are permitted),
    /// again once the airbrakes estimator is retired and it stops, and
    /// throughout the validation deploy.
    pub mpc_predicted_apogee_agl: f32,
    /// Full `NodeStatusMessage` for each node on the bus, as last received.
    pub amp_node: NodeStatusRecord,
    pub icarus_node: NodeStatusRecord,
    pub ozys_node: NodeStatusRecord,
    pub payload_sdrm_node: NodeStatusRecord,
    /// AMP output statuses, 2 bits per output with out1 in the LSBs. Each
    /// pair holds a `PowerOutputStatus` discriminant. From `AmpStatusMessage`,
    /// not the node heartbeat.
    pub amp_out_status: u8,
    /// Shared (AMP) battery voltage.
    pub amp_shared_battery_v: f32,
    /// Payload EPM battery bus voltage (mV), from `CustomPayloadStatusMessage`.
    /// `PAYLOAD_READING_UNAVAILABLE` (0xFFFF) when the payload reported it as
    /// unavailable or has not reported at all — same sentinel as the CAN message.
    pub payload_epm_batt_mv: u16,
    /// Payload EPM switched rail load currents (mA), rail index order 0 `SYS_3V3`,
    /// 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`, 4 `PER_9V`, 5 `PER_12V`.
    pub payload_rail_ma: [u16; 6],
    /// SEM linear actuator positions (steps), experiment channels 1..3.
    pub payload_actuator_steps: [u16; 3],
    pub valid: u8,
}

impl Default for FlightDataSlowRecord {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            temperature: 0.0,
            battery_voltage: 0.0,
            lat_lon: (0.0, 0.0),
            gps_altitude_asl: 0.0,
            num_of_fixed_satalites: 0,
            hdop: 0.0,
            vdop: 0.0,
            pdop: 0.0,
            air_brakes_commanded_extension: f32::NAN,
            air_brakes_actual_extension: f32::NAN,
            air_brakes_servo_temp: f32::NAN,
            air_brakes_validation_deploy: false,
            mpc_predicted_apogee_agl: f32::NAN,
            amp_node: NodeStatusRecord::offline(),
            icarus_node: NodeStatusRecord::offline(),
            ozys_node: NodeStatusRecord::offline(),
            payload_sdrm_node: NodeStatusRecord::offline(),
            amp_out_status: 0,
            amp_shared_battery_v: 0.0,
            payload_epm_batt_mv: PAYLOAD_READING_UNAVAILABLE,
            payload_rail_ma: [PAYLOAD_READING_UNAVAILABLE; 6],
            payload_actuator_steps: [PAYLOAD_READING_UNAVAILABLE; 3],
            valid: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum LogRecord {
    Fast(FlightDataFastRecord),
    Slow(FlightDataSlowRecord),
}

/// Merged view used for CSV export (one row per fast record).
#[derive(Debug, Clone, PartialEq)]
pub struct FlightDataRecord {
    /// [`FlightDataFastRecord::sequence`] — see there for what a discontinuity
    /// means (a drop forwards, a session boundary backwards).
    pub record_count: u32,
    pub timestamp_us: u64,
    /// GPS-disciplined unix clock (µs since epoch), 0 until the clock is ready.
    pub unix_time_us: u64,

    pub acc: [f32; 3],
    pub gyro: [f32; 3],

    pub pressure: f32,

    pub mag: [f32; 3],

    pub deployment_kf_altitude_asl: f32,
    pub deployment_kf_vertical_velocity: f32,
    pub deployment_flags: u8,

    pub airbrakes_kf_altitude_asl: f32,
    pub airbrakes_kf_vertical_velocity: f32,
    pub airbrakes_kf_tilt_deg: f32,
    pub airbrakes_flags: u8,

    /// MS5607 die temperature (C), from the slow snapshot.
    pub temperature: f32,
    pub battery_voltage: f32,

    /// Bitmask for which fields held trustworthy data when logged.
    pub valid: u8,

    pub lat_lon: (f64, f64),
    /// GPS-reported altitude, metres above mean sea level (≈ASL).
    pub gps_altitude_asl: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,

    /// From the fast record, so it is full-rate.
    pub flight_stage: FlightStage,

    /// Bitmask for pyro continuity/fire state (see firmware `ContinuityUpdate`).
    /// Full rate, from the fast record.
    pub pyro_flags: u8,

    /// Commanded extension, from the slow snapshot (the control loop runs at
    /// 10 Hz, so there is nothing faster to log).
    pub air_brakes_commanded_extension: f32,
    /// Icarus-reported extension, from the slow snapshot.
    pub air_brakes_actual_extension: f32,

    /// Airbrakes servo temperature (C), from the slow snapshot.
    pub air_brakes_servo_temp: f32,
    /// The commanded extension is the forced validation deploy rather than the
    /// MPC's output. From the slow snapshot.
    pub air_brakes_validation_deploy: bool,
    /// MPC predicted apogee AGL (m), from the slow snapshot.
    pub mpc_predicted_apogee_agl: f32,

    /// CAN node heartbeats from the slow record.
    pub amp_node: NodeStatusRecord,
    pub icarus_node: NodeStatusRecord,
    pub ozys_node: NodeStatusRecord,
    pub payload_sdrm_node: NodeStatusRecord,
    /// 2 bits per output, out1 in the LSBs (`PowerOutputStatus` discriminants).
    pub amp_out_status: u8,
    pub amp_shared_battery_v: f32,

    /// Payload snapshot from the slow record, in the units the payload CAN
    /// message carries. `PAYLOAD_READING_UNAVAILABLE` (0xFFFF) = no reading.
    pub payload_epm_batt_mv: u16,
    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub payload_rail_ma: [u16; 6],
    /// Experiment channels 1..3.
    pub payload_actuator_steps: [u16; 3],
}

impl FlightDataRecord {
    /// Combine one fast sample with the most recent slow snapshot.
    pub fn from_fast_and_slow(fast: &FlightDataFastRecord, slow: &FlightDataSlowRecord) -> Self {
        Self {
            record_count: fast.sequence,
            timestamp_us: fast.timestamp_us,
            unix_time_us: fast.unix_time_us,
            acc: fast.acc,
            gyro: fast.gyro,
            pressure: fast.pressure,
            mag: fast.mag,
            deployment_kf_altitude_asl: fast.deployment_kf_altitude_asl,
            deployment_kf_vertical_velocity: fast.deployment_kf_vertical_velocity,
            deployment_flags: fast.deployment_flags,
            airbrakes_kf_altitude_asl: fast.airbrakes_kf_altitude_asl,
            airbrakes_kf_vertical_velocity: fast.airbrakes_kf_vertical_velocity,
            airbrakes_kf_tilt_deg: fast.airbrakes_kf_tilt_deg,
            airbrakes_flags: fast.airbrakes_flags,
            temperature: slow.temperature,
            battery_voltage: slow.battery_voltage,
            valid: fast.valid | slow.valid,
            lat_lon: slow.lat_lon,
            gps_altitude_asl: slow.gps_altitude_asl,
            num_of_fixed_satalites: slow.num_of_fixed_satalites,
            hdop: slow.hdop,
            vdop: slow.vdop,
            pdop: slow.pdop,
            flight_stage: fast.flight_stage,
            pyro_flags: fast.pyro_flags,
            air_brakes_commanded_extension: slow.air_brakes_commanded_extension,
            air_brakes_actual_extension: slow.air_brakes_actual_extension,
            air_brakes_servo_temp: slow.air_brakes_servo_temp,
            air_brakes_validation_deploy: slow.air_brakes_validation_deploy,
            mpc_predicted_apogee_agl: slow.mpc_predicted_apogee_agl,
            amp_node: slow.amp_node.clone(),
            icarus_node: slow.icarus_node.clone(),
            ozys_node: slow.ozys_node.clone(),
            payload_sdrm_node: slow.payload_sdrm_node.clone(),
            amp_out_status: slow.amp_out_status,
            amp_shared_battery_v: slow.amp_shared_battery_v,
            payload_epm_batt_mv: slow.payload_epm_batt_mv,
            payload_rail_ma: slow.payload_rail_ma,
            payload_actuator_steps: slow.payload_actuator_steps,
        }
    }
}

/// Expand a tagged log into merged rows (one CSV row per fast sample).
#[cfg(any(feature = "std", test))]
pub fn merge_log_records(log: &[LogRecord]) -> std::vec::Vec<FlightDataRecord> {
    let mut slow = FlightDataSlowRecord::default();
    let mut out = std::vec::Vec::new();
    for rec in log {
        match rec {
            LogRecord::Slow(s) => slow = s.clone(),
            LogRecord::Fast(fast) => out.push(FlightDataRecord::from_fast_and_slow(fast, &slow)),
        }
    }
    out
}

pub const VALID_IMU: u8 = 1 << 0;
// Bit 1 was VALID_BARO, dropped: every published sample carries a baro
// reading, so it could only ever read 1.
pub const VALID_MAG: u8 = 1 << 2;
pub const VALID_GPS_FIX: u8 = 1 << 3;
pub const VALID_GPS_ALT: u8 = 1 << 4;
pub const VALID_BATTERY: u8 = 1 << 5;
// Bits 6-7 unallocated.

/// `deployment_flags` bits — the deployment estimator's status.
///
/// Both bits describe **this record's sample**, not a running state: they are
/// read in the same critical section as the estimator update that produced
/// them, so a single-sample event cannot be missed or land on the wrong row.
/// Both read 0 through Mach lockout, where the KF is frozen and nothing is
/// fused at all.
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

/// `airbrakes_flags` bits — the airbrakes estimator's status.
///
/// The mach-lockout exit is a single drag measurement (the drag-inverted
/// airspeed below Mach 0.8, sustained 1 s), so there is one bit for it;
/// logging it per sample reconstructs the exit post-flight. Bit 2 is free.
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
pub const AIRBRAKES_APOGEE: u8 = 1 << 4;
/// This sample ended a rejection run by re-anchoring: altitude snapped to the
/// baro and velocity uncertainty was re-opened. Set together with
/// `AIRBRAKES_BARO_GATE_REJECT`. A run that ends without this bit is the gate
/// doing its job; a run that ends with it is a diverged filter.
pub const AIRBRAKES_BARO_RESYNC: u8 = 1 << 5;
// bits 6-7 unallocated.

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
