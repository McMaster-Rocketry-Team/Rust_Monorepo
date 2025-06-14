use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::VLPUplinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceToReset {
    All,
    VoidLake,
    AMP,
    AMPOut1,
    AMPOut2,
    AMPOut3,
    AMPOut4,
    Icarus,
    PayloadActivationPCB,
    RocketWifi,
    OzysAll,
    MainBulkhead,
    DrogueBulkhead,
    PayloadEPS1,
    PayloadEPS2,
    AeroRust,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct ResetPacket {
    #[packed_field(element_size_bits = "8", ty = "enum")]
    pub device: DeviceToReset,
}

impl Into<VLPUplinkPacket> for ResetPacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::Reset(self)
    }
}
