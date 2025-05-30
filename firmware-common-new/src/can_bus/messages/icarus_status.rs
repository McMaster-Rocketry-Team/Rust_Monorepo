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
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct IcarusStatusMessage {
    /// Unit: 0.01 inch, e.g. 10 = 0.1 inch
    extended_inches_raw: u16,
    /// Unit: 0.01A, e.g. 10 = 0.1A
    servo_current_raw: u16,
    /// Unit: deg/s
    pub servo_angular_velocity: i16,
}

impl IcarusStatusMessage {
    pub fn new(extended_inches: f32, servo_current: f32, servo_angular_velocity: i16) -> Self {
        Self {
            extended_inches_raw: (extended_inches * 100.0) as u16,
            servo_current_raw: (servo_current * 100.0) as u16,
            servo_angular_velocity,
        }
    }

    pub fn extended_inches(&self) -> f32 {
        self.extended_inches_raw as f32 / 100.0
    }

    pub fn servo_current(&self) -> f32 {
        self.servo_current_raw as f32 / 100.0
    }
}

impl CanBusMessage for IcarusStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for IcarusStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::IcarusStatus(self)
    }
}
