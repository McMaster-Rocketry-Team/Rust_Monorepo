use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "55")]
#[repr(C)]
pub struct RocketStateMessage {
    orientation: [u32; 4],
    velocity: [u32; 3],
    altitude: u32,
    drag_coefficients: [u32; 4],

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl RocketStateMessage {
    pub fn new(
        timestamp_us: u64,
        orientation: &[f32; 4],
        velocity: &[f32; 3],
        altitude: f32,
        drag_coefficients: &[f32; 4],
    ) -> Self {
        Self {
            orientation: orientation.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            velocity: velocity.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            altitude: u32::from_be_bytes(altitude.to_be_bytes()),
            drag_coefficients: drag_coefficients.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            timestamp_us,
        }
    }

    pub fn orientation(&self) -> [f32; 4] {
        self.orientation.map(f32::from_bits)
    }

    pub fn velocity(&self) -> [f32; 3] {
        self.velocity.map(f32::from_bits)
    }

    pub fn altitude(&self) -> f32 {
        f32::from_bits(self.altitude)
    }

    pub fn drag_coefficients(&self) -> [f32; 4] {
        self.drag_coefficients.map(f32::from_bits)
    }
}

impl CanBusMessage for RocketStateMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for RocketStateMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::RocketState(self)
    }
}
