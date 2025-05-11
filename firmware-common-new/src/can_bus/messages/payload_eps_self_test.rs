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

impl PayloadEPSSelfTestMessage {
    pub fn new(
        battery1_ok: bool,
        battery2_ok: bool,
        out_3v3_ok: bool,
        out_5v_ok: bool,
        out_9v_ok: bool,
    ) -> Self {
        Self {
            battery1_ok,
            battery2_ok,
            out_3v3_ok,
            out_5v_ok,
            out_9v_ok,
        }
    }
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
