use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::messages::amp_overwrite::PowerOutputOverwrite;

use super::VLPUplinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
pub struct PayloadEPSOutputOverwritePacket {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub eps1_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub eps1_5v: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub eps1_9v: PowerOutputOverwrite,

    #[packed_field(bits = "6..8", ty = "enum")]
    pub eps2_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "8..10", ty = "enum")]
    pub eps2_5v: PowerOutputOverwrite,
    #[packed_field(bits = "10..12", ty = "enum")]
    pub eps2_9v: PowerOutputOverwrite,
}

impl Into<VLPUplinkPacket> for PayloadEPSOutputOverwritePacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::PayloadEPSOutputOverwrite(self)
    }
}
