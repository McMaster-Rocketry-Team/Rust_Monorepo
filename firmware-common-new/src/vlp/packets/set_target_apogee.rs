use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

use super::VLPUplinkPacket;

fixed_point_factory!(TargetApogeeAglFac, f32, 0.0, 10_000.0, 0.01);

#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
pub struct SetTargetApogeePacket {
    #[packed_field(element_size_bits = "20")]
    pub altitude_agl: Integer<TargetApogeeAglFacBase, packed_bits::Bits<TARGET_APOGEE_AGL_FAC_BITS>>,
}

impl SetTargetApogeePacket {
    pub fn new(target_apogee_agl: f32) -> Self {
        Self {
            altitude_agl: TargetApogeeAglFac::to_fixed_point_capped(target_apogee_agl),
        }
    }

    pub fn get_altitude_agl(&self) -> f32 {
        TargetApogeeAglFac::to_float(self.altitude_agl)
    }
}

impl Into<VLPUplinkPacket> for SetTargetApogeePacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::SetTargetApogee(self)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SetTargetApogeePacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "SetTargetApogeePacket")
    }
}
