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
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct AmpStatusMessage {
    pub shared_battery_mv: u16,
    #[packed_field(bits = "16..18", ty = "enum")]
    pub out1: PowerOutputStatus,
    #[packed_field(bits = "18..20", ty = "enum")]
    pub out2: PowerOutputStatus,
    #[packed_field(bits = "20..22", ty = "enum")]
    pub out3: PowerOutputStatus,
    #[packed_field(bits = "22..24", ty = "enum")]
    pub out4: PowerOutputStatus,
}

impl AmpStatusMessage {
    pub fn new(
        shared_battery_mv: u16,
        out1: PowerOutputStatus,
        out2: PowerOutputStatus,
        out3: PowerOutputStatus,
        out4: PowerOutputStatus,
    ) -> Self {
        Self {
            shared_battery_mv,
            out1,
            out2,
            out3,
            out4,
        }
    }
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
