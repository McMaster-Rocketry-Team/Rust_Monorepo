use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum TempuratureSource {
    STM32 = 0,
    Barometer = 1,
    Servo = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "11")]
#[repr(C)]
pub struct TempuratureMeasurementMessage {
    /// Unit: 0.1C, e.g. 250 = 25C
    pub temperature: u16,

    /// Measurement timestamp, milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(element_size_bits = "48")]
    pub timestamp: u64,

    #[packed_field(bits = "64..66", ty = "enum")]
    pub source: TempuratureSource,
}

impl TempuratureMeasurementMessage {
    pub fn new(timestamp: f64, temperature: u16, source: TempuratureSource) -> Self {
        Self {
            temperature,
            source,
            timestamp: (timestamp as u64).into(),
        }
    }
}

impl CanBusMessage for TempuratureMeasurementMessage {
    fn priority(&self) -> u8 {
        5
    }
}