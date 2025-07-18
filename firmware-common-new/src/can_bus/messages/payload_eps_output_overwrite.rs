use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum, amp_overwrite::PowerOutputOverwrite};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct PayloadEPSOutputOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out_5v: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out_9v: PowerOutputOverwrite,

    /// Node ID of EPS to control
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl CanBusMessage for PayloadEPSOutputOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSOutputOverwriteMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSOutputOverwrite(self)
    }
}
