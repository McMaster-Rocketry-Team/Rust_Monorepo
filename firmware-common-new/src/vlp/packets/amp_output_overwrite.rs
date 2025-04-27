use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::messages::payload_eps_output_overwrite::PowerOutputOverwrite;

use super::VLPUplinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct AMPOutputOverwritePacket {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out1: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out2: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out3: PowerOutputOverwrite,
    #[packed_field(bits = "6..8", ty = "enum")]
    pub out4: PowerOutputOverwrite,
}

impl Into<VLPUplinkPacket> for AMPOutputOverwritePacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::AMPOutputOverwrite(self)
    }
}
