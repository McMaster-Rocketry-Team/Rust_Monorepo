#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "wasm")]
use tsify::Tsify;

use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "13")]
#[repr(C)]
pub struct BaroMeasurementMessage {
    pressure_raw: u32,

    /// Unit: 0.1C, e.g. 250 = 25C
    temperature_raw: u16,

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl BaroMeasurementMessage {
    pub fn new(timestamp_us: u64, pressure: f32, temperature: f32) -> Self {
        Self {
            pressure_raw: u32::from_be_bytes(pressure.to_be_bytes()),
            temperature_raw: (temperature * 10.0) as u16,
            timestamp_us,
        }
    }

    /// Pressure in Pa
    pub fn pressure(&self) -> f32 {
        f32::from_bits(self.pressure_raw)
    }

    /// Temperature in C
    pub fn temperature(&self) -> f32 {
        self.temperature_raw as f32 / 10.0
    }

    pub fn altitude(&self) -> f32 {
        return calculate_isa_altitude(Pascals(self.pressure() as f64)).0 as f32;
    }
}

impl CanBusMessage for BaroMeasurementMessage {
    fn priority(&self) -> u8 {
        3
    }
}

impl Into<CanBusMessageEnum> for BaroMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::BaroMeasurement(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::init_logger;

    #[test]
    fn altitude_calculation(){
        init_logger();

        log_info!("{}", BaroMeasurementMessage::new(0, 103325.3, 30.0).altitude())
    }
}