use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "12")]
#[repr(C)]
pub struct BaroMeasurementMessage {
    pressure: u32,

    /// Unit: 0.1C, e.g. 250 = 25C
    temperature: u16,

    /// Measurement timestamp, milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(element_size_bits = "48")]
    pub timestamp: u64,
}

impl BaroMeasurementMessage {
    pub fn new(timestamp: f64, pressure: f32, temperature: f32) -> Self {
        Self {
            pressure: u32::from_be_bytes(pressure.to_be_bytes()),
            temperature: (temperature * 10.0) as u16,
            timestamp: (timestamp as u64).into(),
        }
    }

    /// Pressure in Pa
    pub fn pressure(&self) -> f32 {
        f32::from_bits(self.pressure)
    }

    /// Temperature in C
    pub fn temperature(&self) -> f32 {
        self.temperature as f32 / 10.0
    }

    pub fn altitude(&self) -> f32 {
        return calculate_isa_altitude(Pascals(self.pressure as f64)).0 as f32;
    }
}

impl CanBusMessage for BaroMeasurementMessage {
    fn priority(&self) -> u8 {
        3
    }
}
