use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpControlMessage {
    pub out1_enable: bool,
    pub out2_enable: bool,
    pub out3_enable: bool,
    pub out4_enable: bool,
}

impl CanBusMessage for AmpControlMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AmpControlMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpControl(self)
    }
}
