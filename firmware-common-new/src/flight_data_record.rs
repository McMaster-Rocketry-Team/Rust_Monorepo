use crate::can_bus::messages::{
    custom_payload_status::PAYLOAD_READING_UNAVAILABLE, vl_status::FlightStage,
};

/// High-rate IMU / baro / mag / estimator / pyro / airbrakes-extension sample.
pub const RECORD_TAG_FAST: u8 = 0x01;
/// Low-rate GPS / battery / AMP / servo-temperature snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataFastRecord {
    /// Per-record counter, restarting at 0 each armed session — the logger runs
    /// inside armed mode, so a re-arm (or a reboot) begins a new count.
    ///
    /// Within a session, a step of anything other than +1 means records were
    /// dropped on the way to the SD card. A step *backwards* is a session
    /// boundary, not a drop: one stored log can hold several armed sessions
    /// (nothing is logged in between), and this is what marks where one ends.
    pub sequence: u32,
    pub timestamp_us: u64,
    /// GPS-disciplined unix clock, microseconds since the epoch. 0 until the
    /// clock is ready (no time fix yet).
    pub unix_time_us: u64,
    pub acc: [f32; 3],
    pub gyro: [f32; 3],
    pub temperature: f32,
    pub pressure: f32,
    pub mag: [f32; 3],
    /// Deployment (slow baro) estimator altitude ASL (m). NaN until the
    /// estimator has run.
    pub kf_altitude_asl: f32,
    /// Deployment (slow baro) estimator vertical velocity (m/s). NaN until
    /// the estimator has run.
    pub kf_vertical_velocity: f32,
    /// Airbrakes estimator altitude ASL (m). NaN until it has a value.
    pub ab_altitude_asl: f32,
    /// Airbrakes estimator vertical velocity (m/s). NaN until its vertical
    /// filter is born.
    pub ab_vertical_velocity: f32,
    /// Airbrakes estimator tilt from vertical (deg). NaN before ignition.
    pub ab_tilt_deg: f32,
    /// Airbrakes estimator status bits (`AB_*` consts): drag check, burnout
    /// latch, filter-born, apogee.
    pub ab_flags: u8,
    /// Mirror of the deployment estimator's `RocketState` (plus the device
    /// modes), with its Mach lockout folded into `Ascent`. The chutes'
    /// `deployed` bools ride in this record's `pyro_flags` fire bits.
    pub flight_stage: FlightStage,
    /// Bitmask for pyro continuity/fire state (`PYRO_*` consts). Logged at
    /// the full fast rate so pyro fire edges are timestamped to ±2.3 ms.
    pub pyro_flags: u8,
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended. NaN until
    /// the firmware has commanded anything (i.e. outside Armed and Demo).
    pub air_brakes_commanded_extension: f32,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
    /// NaN until Icarus reports — which is the interesting case, since an
    /// Icarus that is offline or silent would otherwise be indistinguishable
    /// from one reporting fully-stowed brakes.
    pub air_brakes_actual_extension: f32,
    pub valid: u8,
}

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataSlowRecord {
    pub timestamp_us: u64,
    pub battery_voltage: f32,
    pub lat_lon: (f64, f64),
    /// GPS-reported altitude, metres above mean sea level (≈ASL).
    pub gps_altitude_asl: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,
    pub flight_stage: FlightStage,
    /// Airbrakes servo temperature (C) reported by Icarus. NaN until Icarus
    /// reports, for the same reason as `air_brakes_actual_extension`.
    pub air_brakes_servo_temp: f32,
    /// AMP node reachable over the CAN bus.
    pub amp_online: bool,
    /// AMP output statuses, 2 bits per output with out1 in the LSBs. Each
    /// pair holds a `PowerOutputStatus` discriminant.
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
            battery_voltage: 0.0,
            lat_lon: (0.0, 0.0),
            gps_altitude_asl: 0.0,
            num_of_fixed_satalites: 0,
            hdop: 0.0,
            vdop: 0.0,
            pdop: 0.0,
            flight_stage: FlightStage::LowPower,
            air_brakes_servo_temp: f32::NAN,
            amp_online: false,
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

    pub temperature: f32,
    pub pressure: f32,

    pub mag: [f32; 3],

    pub kf_altitude_asl: f32,
    pub kf_vertical_velocity: f32,

    pub ab_altitude_asl: f32,
    pub ab_vertical_velocity: f32,
    pub ab_tilt_deg: f32,
    pub ab_flags: u8,

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

    /// Full-rate stage from the fast record.
    pub flight_stage: FlightStage,

    /// Bitmask for pyro continuity/fire state (see firmware `ContinuityUpdate`).
    /// Full rate, from the fast record.
    pub pyro_flags: u8,

    /// Full rate, from the fast record.
    pub air_brakes_commanded_extension: f32,
    pub air_brakes_actual_extension: f32,

    /// Airbrakes servo temperature (C), from the slow snapshot.
    pub air_brakes_servo_temp: f32,

    /// AMP snapshot from the slow record.
    pub amp_online: bool,
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
            temperature: fast.temperature,
            pressure: fast.pressure,
            mag: fast.mag,
            kf_altitude_asl: fast.kf_altitude_asl,
            kf_vertical_velocity: fast.kf_vertical_velocity,
            ab_altitude_asl: fast.ab_altitude_asl,
            ab_vertical_velocity: fast.ab_vertical_velocity,
            ab_tilt_deg: fast.ab_tilt_deg,
            ab_flags: fast.ab_flags,
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
            air_brakes_commanded_extension: fast.air_brakes_commanded_extension,
            air_brakes_actual_extension: fast.air_brakes_actual_extension,
            air_brakes_servo_temp: slow.air_brakes_servo_temp,
            amp_online: slow.amp_online,
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
pub const VALID_BARO: u8 = 1 << 1;
pub const VALID_MAG: u8 = 1 << 2;
pub const VALID_GPS_FIX: u8 = 1 << 3;
pub const VALID_GPS_ALT: u8 = 1 << 4;
pub const VALID_BATTERY: u8 = 1 << 5;
// Bits 6-7 unallocated.

/// `ab_flags` bits — the airbrakes estimator's status.
///
/// The mach-lockout exit is a single drag measurement (the drag-inverted
/// airspeed below Mach 0.8, sustained 1 s), so there is one bit for it;
/// logging it per sample reconstructs the exit post-flight. Bit 2 is free.
pub const AB_SUBSONIC_DRAG: u8 = 1 << 0;
/// The axial-sign burnout latch has fired: the motor is out and the drag
/// channel is honest. Nothing can birth the vertical filter before this, on
/// either the supersonic or the subsonic path, so it separates "the brakes
/// never opened because the motor never looked out" from "because the drag
/// check never passed".
pub const AB_BURNOUT: u8 = 1 << 1;
/// The vertical filter is born (baro trusted; MPC state is live).
pub const AB_BARO_TRUSTED: u8 = 1 << 3;
pub const AB_APOGEE: u8 = 1 << 4;
// bits 2 and 5-7 unallocated.

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
