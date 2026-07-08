use crate::can_bus::messages::vl_status::FlightStage;

/// Legacy v1 on-disk tag (fixed 112-byte [`FlightDataRecord`] blob).
pub const RECORD_TAG_V1: u8 = 0x00;
/// High-rate IMU / baro / mag sample.
pub const RECORD_TAG_IMU: u8 = 0x01;
/// Low-rate GPS / battery / pyro / flight-stage snapshot.
pub const RECORD_TAG_SLOW: u8 = 0x02;

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataImuRecord {
    pub sequence: u32,
    pub timestamp_us: u64,
    pub acc: [f32; 3],
    pub gyro: [f32; 3],
    pub temperature: f32,
    pub pressure: f32,
    pub mag: [f32; 3],
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
            valid: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum LogRecord {
    Imu(FlightDataImuRecord),
    Slow(FlightDataSlowRecord),
}

/// Merged view used for CSV export and v1 compatibility.
#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, PartialEq)]
pub struct FlightDataRecord {
    pub record_count: u32,
    pub timestamp_us: u64,

    pub acc: [f32; 3],
    pub gyro: [f32; 3],

    pub temperature: f32,
    pub pressure: f32,

    pub mag: [f32; 3],

    pub battery_voltage: f32,

    /// Bitmask for which fields held trustworthy data when logged.
    pub valid: u8,

    pub lat_lon: (f64, f64),
    pub altitude: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,

    pub flight_stage: FlightStage,

    /// Bitmask for pyro continuity/fire state (see firmware `ContinuityUpdate`).
    pub pyro_flags: u8,
}

impl FlightDataRecord {
    /// Combine one IMU sample with the most recent slow snapshot.
    pub fn from_imu_and_slow(imu: &FlightDataImuRecord, slow: &FlightDataSlowRecord) -> Self {
        Self {
            record_count: imu.sequence,
            timestamp_us: imu.timestamp_us,
            acc: imu.acc,
            gyro: imu.gyro,
            temperature: imu.temperature,
            pressure: imu.pressure,
            mag: imu.mag,
            battery_voltage: slow.battery_voltage,
            valid: imu.valid | slow.valid,
            lat_lon: slow.lat_lon,
            altitude: slow.altitude,
            num_of_fixed_satalites: slow.num_of_fixed_satalites,
            hdop: slow.hdop,
            vdop: slow.vdop,
            pdop: slow.pdop,
            flight_stage: slow.flight_stage,
            pyro_flags: slow.pyro_flags,
        }
    }
}

/// Expand a tagged v2 log into merged rows (one CSV row per IMU sample).
#[cfg(any(feature = "std", test))]
pub fn merge_log_records(log: &[LogRecord]) -> std::vec::Vec<FlightDataRecord> {
    let mut slow = FlightDataSlowRecord::default();
    let mut out = std::vec::Vec::new();
    for rec in log {
        match rec {
            LogRecord::Slow(s) => slow = s.clone(),
            LogRecord::Imu(imu) => out.push(FlightDataRecord::from_imu_and_slow(imu, &slow)),
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

pub const PYRO_MAIN_CONTINUITY: u8 = 1 << 0;
pub const PYRO_MAIN_FIRE: u8 = 1 << 1;
pub const PYRO_DROGUE_CONTINUITY: u8 = 1 << 2;
pub const PYRO_DROGUE_FIRE: u8 = 1 << 3;
pub const PYRO_SHORT_CIRCUIT: u8 = 1 << 4;
