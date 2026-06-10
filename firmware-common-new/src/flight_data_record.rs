use crate::{
    can_bus::messages::vl_status::FlightStage, gps::GPSData,
    vlp::packets::gps_beacon::GPSBeaconPacket,
};

#[derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)]
pub struct FlightDataRecord {
    pub record_count: u32,
    // theoretically, since the recording will start at boot, and we know boot time, we may not need to log the timestamp, this is just in case.
    pub timestamp_us: u64,

    // [f32;3] is used because Vector3<f32> wouldve been a pain to make serialisable for rkyv
    // IMU data
    pub acc: [f32; 3],
    pub gyro: [f32; 3],

    // Barometer data
    pub temperature: f32,
    pub pressure: f32,
    ///
    pub mag: [f32; 3],
    ///
    pub battery_voltage: f32,
    ///
    // bitmask for if each of the data points is valid, (exists cause GPS data is inconsistent and rkyv cant work with option)
    pub valid: u8,
    ///
    // gps data
    pub lat_lon: (f64, f64),
    pub altitude: f32,
    pub num_of_fixed_satalites: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub pdop: f32,
    ///
    pub flight_stage: FlightStage,

    // bitmask for ContinuityUpdate (which is in vlf5/firmware)
    pub pyro_flags: u8,
}
