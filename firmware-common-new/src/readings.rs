use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BaroData {
    /// C
    pub temperature: f32,
    /// Pa
    pub pressure: f32,
}

impl BaroData {
    pub fn altitude(&self) -> f32 {
        return calculate_isa_altitude(Pascals(self.pressure as f64)).0 as f32;
        // see https://github.com/pimoroni/bmp280-python/blob/master/library/bmp280/__init__.py
        // let air_pressure_hpa = self.pressure / 100.0;
        // return ((powf(1013.25 / air_pressure_hpa, 1.0 / 5.257) - 1.0)
        //     * (self.temperature + 273.15))
        //     / 0.0065;
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IMUData {
    /// m/s^2
    pub acc: [f32; 3],
    /// deg/s
    pub gyro: [f32; 3],
}
