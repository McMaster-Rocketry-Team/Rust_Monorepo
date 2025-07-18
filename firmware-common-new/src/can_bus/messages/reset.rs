use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct ResetMessage {
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
    pub reset_all: bool,
    pub into_bootloader: bool,
}

impl CanBusMessage for ResetMessage {
    fn priority(&self) -> u8 {
        0
    }
}

impl Into<CanBusMessageEnum> for ResetMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::Reset(self)
    }
}
