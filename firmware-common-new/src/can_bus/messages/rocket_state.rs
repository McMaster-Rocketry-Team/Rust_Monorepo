use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "44")]
#[repr(C)]
pub struct RocketStateMessage { 
    /// tilt of the rocket in degrees, 0 - 90
    tilt_deg: u32,
    velocity: [u32; 2],
    altitude_asl: u32,
    launch_pad_altitude_asl: u32,
    drag_coefficients: [u32; 4],

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,

    pub is_coasting: bool,
}

impl RocketStateMessage {
    pub fn new(
        timestamp_us: u64,
        tilt: f32,
        velocity: &[f32; 2],
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
        is_coasting: bool,
        drag_coefficients: &[f32; 4],
    ) -> Self {
        Self {
            tilt_deg: u32::from_be_bytes(tilt.to_be_bytes()),
            velocity: velocity.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            altitude_asl: u32::from_be_bytes(altitude_asl.to_be_bytes()),
            launch_pad_altitude_asl: u32::from_be_bytes(launch_pad_altitude_asl.to_be_bytes()),
            is_coasting,
            drag_coefficients: drag_coefficients.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            timestamp_us,
        }
    }

    /// tilt of the rocket in degrees, 0 - 90
    pub fn tilt_deg(&self) -> f32 {
        f32::from_bits(self.tilt_deg)
    }

    pub fn velocity(&self) -> [f32; 2] {
        self.velocity.map(f32::from_bits)
    }

    pub fn altitude_asl(&self) -> f32 {
        f32::from_bits(self.altitude_asl)
    }

    pub fn launch_pad_altitude_asl(&self) -> f32 {
        f32::from_bits(self.launch_pad_altitude_asl)
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
