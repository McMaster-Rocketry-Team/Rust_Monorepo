use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::VLPUplinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    LowPower = 0,
    SelfTest = 1,
    ReadyToLaunch = 2,
    Landed = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct ChangeModePacket {
    #[packed_field(element_size_bits = "2", ty = "enum")]
    pub mode: Mode,
}

impl Into<VLPUplinkPacket> for ChangeModePacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::ChangeMode(self)
    }
}
