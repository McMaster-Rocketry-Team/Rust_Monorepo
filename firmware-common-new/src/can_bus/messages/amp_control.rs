#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "wasm")]
use tsify::Tsify;

use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpControlMessage {
    pub out1_enable: bool,
    pub out2_enable: bool,
    pub out3_enable: bool,
    pub out4_enable: bool,
}

impl AmpControlMessage {
    pub fn new(out1_enable: bool, out2_enable: bool, out3_enable: bool, out4_enable: bool) -> Self {
        Self {
            out1_enable,
            out2_enable,
            out3_enable,
            out4_enable,
        }
    }
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
