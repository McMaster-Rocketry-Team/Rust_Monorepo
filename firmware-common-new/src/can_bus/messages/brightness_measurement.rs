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
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "11")]
#[repr(C)]
pub struct BrightnessMeasurementMessage {
    brightness_raw: u32,

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

#[cfg_attr(feature = "wasm", wasm_bindgen)]
impl BrightnessMeasurementMessage {
    pub fn new(timestamp_us: u64, brightness: f32) -> Self {
        Self {
            brightness_raw: u32::from_be_bytes(brightness.to_be_bytes()),
            timestamp_us,
        }
    }

    /// Brightness in Lux
    pub fn brightness(&self) -> f32 {
        f32::from_bits(self.brightness_raw)
    }
}

impl CanBusMessage for BrightnessMeasurementMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for BrightnessMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::BrightnessMeasurement(self)
    }
}
