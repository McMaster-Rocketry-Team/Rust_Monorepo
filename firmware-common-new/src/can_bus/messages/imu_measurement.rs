use nalgebra::Vector3;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "31")]
#[repr(C)]
pub struct IMUMeasurementMessage {
    acc_raw: [u32; 3],
    gyro_raw: [u32; 3],

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl IMUMeasurementMessage {
    pub fn new(timestamp_us: u64, acc: &Vector3<f32>, gyro: &Vector3<f32>) -> Self {
        Self {
            acc_raw: vec3_to_u32_array(acc),
            gyro_raw: vec3_to_u32_array(gyro),
            timestamp_us,
        }
    }

    /// Acceleration in m/s^2
    pub fn acc(&self) -> Vector3<f32> {
        Vector3::from_column_slice(&self.acc_raw.map(f32::from_bits))
    }

    /// Gyroscope in deg/s
    pub fn gyro(&self) -> Vector3<f32> {
        Vector3::from_column_slice(&self.gyro_raw.map(f32::from_bits))
    }
}

fn vec3_to_u32_array(vec: &Vector3<f32>) -> [u32;3]{
    let mut result = [0u32;3];

    result[0] = u32::from_be_bytes(vec.x.to_be_bytes());
    result[1] = u32::from_be_bytes(vec.y.to_be_bytes());
    result[2] = u32::from_be_bytes(vec.z.to_be_bytes());

    result
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
