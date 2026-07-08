use crate::can_bus::messages::vl_status::FlightStage;

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
