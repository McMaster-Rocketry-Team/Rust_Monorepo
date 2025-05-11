#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "wasm")]
use tsify::Tsify;

use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "31")]
#[repr(C)]
pub struct IMUMeasurementMessage {
    acc: [u32; 3],
    gyro: [u32; 3],

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl IMUMeasurementMessage {
    pub fn new(timestamp_us: u64, acc: [f32; 3], gyro: [f32; 3]) -> Self {
        Self {
            acc: acc.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            gyro: gyro.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            timestamp_us,
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
    fn priority(&self) -> u8 {
        3
    }
}

impl Into<CanBusMessageEnum> for IMUMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::IMUMeasurement(self)
    }
}
