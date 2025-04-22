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
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct TempuratureMeasurementMessage {
    /// Unit: 0.1C, e.g. 250 = 25C
    pub temperature: u16,
    #[packed_field(bits = "16..18", ty = "enum")]
    pub source: TempuratureSource,
}

impl TempuratureMeasurementMessage {
    pub fn new(temperature: u16, source: TempuratureSource) -> Self {
        Self {
            temperature,
            source,
        }
    }
}

impl CanBusMessage for TempuratureMeasurementMessage {
    fn len() -> usize {
        3
    }

    fn priority(&self) -> u8 {
        5
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}