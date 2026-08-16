use crate::can_bus::messages::vl_status::FlightStage;

/// High-rate IMU / baro / mag / estimator / pyro / airbrakes-extension sample.
pub const RECORD_TAG_FAST: u8 = 0x01;
/// Low-rate GPS / battery / AMP / servo-temperature snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataFastRecord {
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
    /// Airbrakes estimator status bits (`AB_*` consts): the three
    /// lockout-exit votes, filter-born, apogee.
    pub ab_flags: u8,
    /// Honest mirror of the deployment estimator's `RocketState` (plus the
    /// device modes). The chutes' `deployed` bools ride in this record's
    /// `pyro_flags` fire bits.
    pub flight_stage: FlightStage,
    /// Bitmask for pyro continuity/fire state (`PYRO_*` consts). Logged at
    /// the full fast rate so pyro fire edges are timestamped to ±2.3 ms.
    pub pyro_flags: u8,
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended.
    pub air_brakes_commanded_extension: f32,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
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
    /// Airbrakes servo temperature (C) reported by Icarus.
    pub air_brakes_servo_temp: f32,
    /// AMP node reachable over the CAN bus.
    pub amp_online: bool,
    /// AMP output statuses, 2 bits per output with out1 in the LSBs. Each
    /// pair holds a `PowerOutputStatus` discriminant.
    pub amp_out_status: u8,
    /// Shared (AMP) battery voltage.
    pub amp_shared_battery_v: f32,
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
            air_brakes_servo_temp: 0.0,
            amp_online: false,
            amp_out_status: 0,
            amp_shared_battery_v: 0.0,
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
pub const VALID_AIRBRAKES_COMMANDED: u8 = 1 << 6;
pub const VALID_AIRBRAKES_ACTUAL: u8 = 1 << 7;

/// `ab_flags` bits — the airbrakes estimator's status. The three vote bits
/// are the lockout-exit votes (2-of-3 sustained opens the lockout); logging
/// them per sample reconstructs the exit truth table post-flight.
pub const AB_VOTE_INERTIAL: u8 = 1 << 0;
pub const AB_VOTE_DEPLOYMENT: u8 = 1 << 1;
pub const AB_VOTE_BARO_RATE: u8 = 1 << 2;
/// The vertical filter is born (baro trusted; MPC state is live).
pub const AB_BARO_TRUSTED: u8 = 1 << 3;
pub const AB_APOGEE: u8 = 1 << 4;
// bits 5-7 unallocated.

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
