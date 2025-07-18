use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb")]
#[repr(C)]
pub struct PayloadEPSSelfTestMessage {
    #[packed_field(bits = "0")]
    pub battery1_ok: bool,
    pub battery2_ok: bool,
    pub out_3v3_ok: bool,
    pub out_5v_ok: bool,
    pub out_9v_ok: bool,
}

impl CanBusMessage for PayloadEPSSelfTestMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSSelfTestMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSSelfTest(self)
    }
}
