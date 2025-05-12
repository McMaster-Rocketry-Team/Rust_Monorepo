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
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputStatus {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bits = "3")]
#[repr(C)]
pub struct AmpOutputStatus {
    #[packed_field(bits = "0..1")]
    pub overwrote: bool,
    #[packed_field(bits = "1..3", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct AmpStatusMessage {
    pub shared_battery_mv: u16,
    #[packed_field(element_size_bits = "3")]
    pub out1: AmpOutputStatus,
    #[packed_field(element_size_bits = "3")]
    pub out2: AmpOutputStatus,
    #[packed_field(element_size_bits = "3")]
    pub out3: AmpOutputStatus,
    #[packed_field(element_size_bits = "3")]
    pub out4: AmpOutputStatus,
}

impl CanBusMessage for AmpStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for AmpStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpStatus(self)
    }
}
