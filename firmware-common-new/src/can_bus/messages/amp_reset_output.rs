use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpResetOutputMessage {
    // 1, 2, 3, 4
    pub output: u8,
}

impl CanBusMessage for AmpResetOutputMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AmpResetOutputMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpResetOutput(self)
    }
}
