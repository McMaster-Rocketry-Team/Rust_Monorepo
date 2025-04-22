use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "30")]
#[repr(C)]
pub struct IMUMeasurementMessage {
    acc: [u32; 3],
    gyro: [u32; 3],

    /// Measurement timestamp, milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(element_size_bits = "48")]
    pub timestamp: u64,
}

impl IMUMeasurementMessage {
    pub fn new(timestamp: f64, acc: [f32; 3], gyro: [f32; 3]) -> Self {
        Self {
            acc: acc.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            gyro: gyro.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            timestamp: (timestamp as u64).into(),
        }
    }

    /// Acceleration in m/s^2
    pub fn acc(&self) -> [f32; 3] {
        self.acc.map(f32::from_bits)
    }

    /// Gyroscope in deg/s
    pub fn gyro(&self) -> [f32; 3] {
        self.gyro.map(f32::from_bits)
    }
}

impl CanBusMessage for IMUMeasurementMessage {
    fn len() -> usize {
        30
    }

    fn priority(&self) -> u8 {
        6
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
