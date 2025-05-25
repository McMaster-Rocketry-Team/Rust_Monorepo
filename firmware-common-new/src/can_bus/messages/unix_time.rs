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
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "7")]
#[repr(C)]
pub struct UnixTimeMessage {
    /// Current microseconds since Unix epoch, floored to the nearest us
    /// 56 representation of it will overflow at year 4254
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl CanBusMessage for UnixTimeMessage {
    fn priority(&self) -> u8 {
        1
    }
}

impl Into<CanBusMessageEnum> for UnixTimeMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::UnixTime(self)
    }
}

