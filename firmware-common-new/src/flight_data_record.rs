use crate::can_bus::messages::vl_status::FlightStage;

/// High-rate IMU / baro / mag / estimator sample.
pub const RECORD_TAG_FAST: u8 = 0x01;
/// Low-rate GPS / battery / pyro / airbrakes snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataFastRecord {
    pub sequence: u32,
    pub timestamp_us: u64,
    pub acc: [f32; 3],
    pub gyro: [f32; 3],
    pub temperature: f32,
    pub pressure: f32,
    pub mag: [f32; 3],
    /// State-estimator KF altitude ASL (m). NaN until the estimator has run.
    pub kf_altitude_asl: f32,
    /// State-estimator KF vertical velocity (m/s). NaN until the estimator has run.
    pub kf_vertical_velocity: f32,
    pub flight_stage: FlightStage,
    pub valid: u8,
}

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataSlowRecord {
    pub timestamp_us: u64,
    pub battery_voltage: f32,
    pub lat_lon: (f64, f64),
    pub altitude: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,
    pub flight_stage: FlightStage,
    pub pyro_flags: u8,
    /// Commanded extension, 0.0 = retracted, 1.0 = fully extended.
    pub air_brakes_commanded_extension: f32,
    /// Reported extension from Icarus, 0.0 = retracted, 1.0 = fully extended.
    pub air_brakes_actual_extension: f32,
    pub valid: u8,
}

impl Default for FlightDataSlowRecord {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            battery_voltage: 0.0,
            lat_lon: (0.0, 0.0),
            altitude: 0.0,
            num_of_fixed_satalites: 0,
            hdop: 0.0,
            vdop: 0.0,
            pdop: 0.0,
            flight_stage: FlightStage::LowPower,
            pyro_flags: 0,
            air_brakes_commanded_extension: 0.0,
            air_brakes_actual_extension: 0.0,
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

    pub acc: [f32; 3],
    pub gyro: [f32; 3],

    pub temperature: f32,
    pub pressure: f32,

    pub mag: [f32; 3],

    pub kf_altitude_asl: f32,
    pub kf_vertical_velocity: f32,

    pub battery_voltage: f32,

    /// Bitmask for which fields held trustworthy data when logged.
    pub valid: u8,

    pub lat_lon: (f64, f64),
    pub altitude: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,

    /// Full-rate stage from the fast record.
    pub flight_stage: FlightStage,

    /// Bitmask for pyro continuity/fire state (see firmware `ContinuityUpdate`).
    pub pyro_flags: u8,

    pub air_brakes_commanded_extension: f32,
    pub air_brakes_actual_extension: f32,
}

impl FlightDataRecord {
    /// Combine one fast sample with the most recent slow snapshot.
    pub fn from_fast_and_slow(fast: &FlightDataFastRecord, slow: &FlightDataSlowRecord) -> Self {
        Self {
            record_count: fast.sequence,
            timestamp_us: fast.timestamp_us,
            acc: fast.acc,
            gyro: fast.gyro,
            temperature: fast.temperature,
            pressure: fast.pressure,
            mag: fast.mag,
            kf_altitude_asl: fast.kf_altitude_asl,
            kf_vertical_velocity: fast.kf_vertical_velocity,
            battery_voltage: slow.battery_voltage,
            valid: fast.valid | slow.valid,
            lat_lon: slow.lat_lon,
            altitude: slow.altitude,
            num_of_fixed_satalites: slow.num_of_fixed_satalites,
            hdop: slow.hdop,
            vdop: slow.vdop,
            pdop: slow.pdop,
            flight_stage: fast.flight_stage,
            pyro_flags: slow.pyro_flags,
            air_brakes_commanded_extension: slow.air_brakes_commanded_extension,
            air_brakes_actual_extension: slow.air_brakes_actual_extension,
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

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
