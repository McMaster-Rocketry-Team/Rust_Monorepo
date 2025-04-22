use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
#[repr(C)]
pub struct BrightnessMeasurementMessage {
    brightness: u32,

    /// Measurement timestamp, milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(element_size_bits = "48")]
    pub timestamp: u64,
}

impl BrightnessMeasurementMessage {
    pub fn new(timestamp: f64, brightness: f32) -> Self {
        Self {
            brightness: u32::from_be_bytes(brightness.to_be_bytes()),
            timestamp: (timestamp as u64).into(),
        }
    }

    /// Brightness in Lux
    pub fn brightness(&self) -> f32 {
        f32::from_bits(self.brightness)
    }
}

impl CanBusMessage for BrightnessMeasurementMessage {
    fn len() -> usize {
        10
    }

    fn priority(&self) -> u8 {
        6
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..Self::len()]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
