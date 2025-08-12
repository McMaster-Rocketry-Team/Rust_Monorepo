use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "22")]
#[repr(C)]
pub struct RocketStateMessage {
    velocity_raw: [u32; 2],
    altitude_agl_raw: u32,
    pub ap_residue: i16,

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,

    pub is_coasting: bool,
}

impl RocketStateMessage {
    pub fn new(
        timestamp_us: u64,
        velocity: &[f32; 2],
        altitude_agl: f32,
        ap_residue: i16,
        is_coasting: bool,
    ) -> Self {
        Self {
            velocity_raw: velocity.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            altitude_agl_raw: u32::from_be_bytes(altitude_agl.to_be_bytes()),
            ap_residue,
            is_coasting,
            timestamp_us,
        }
    }

    pub fn velocity(&self) -> [f32; 2] {
        self.velocity_raw.map(f32::from_bits)
    }

    pub fn altitude_agl(&self) -> f32 {
        f32::from_bits(self.altitude_agl_raw)
    }
}

impl CanBusMessage for RocketStateMessage {
    fn priority(&self) -> u8 {
        3
    }
}

impl Into<CanBusMessageEnum> for RocketStateMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::RocketState(self)
    }
}
