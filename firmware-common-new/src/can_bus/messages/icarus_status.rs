use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct IcarusStatusMessage {
    /// Unit: 0.01 inch, e.g. 10 = 0.1 inch
    pub extended_inches: u16,
    /// Unit: 0.01A, e.g. 10 = 0.1A
    pub servo_current: u16,
    /// Unit: deg/s
    pub servo_angular_velocity: i16,
}

impl IcarusStatusMessage {
    pub fn new(extended_inches: f32, servo_current: f32, servo_angular_velocity: i16) -> Self {
        Self {
            extended_inches: (extended_inches * 100.0) as u16,
            servo_current: (servo_current * 100.0) as u16,
            servo_angular_velocity,
        }
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
