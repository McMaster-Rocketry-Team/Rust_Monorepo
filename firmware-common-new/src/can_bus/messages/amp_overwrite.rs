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
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputOverwrite {
    NoOverwrite = 0,
    ForceEnabled = 1,
    ForceDisabled = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out1: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out2: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out3: PowerOutputOverwrite,
    #[packed_field(bits = "6..8", ty = "enum")]
    pub out4: PowerOutputOverwrite,
}

impl CanBusMessage for AmpOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AmpOverwriteMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpOverwrite(self)
    }
}
