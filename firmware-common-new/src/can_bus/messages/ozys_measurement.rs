use core::f32;

use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "16")]
#[repr(C)]
pub struct OzysMeasurementMessage {
    sg_1_raw: u32,
    sg_2_raw: u32,
    sg_3_raw: u32,
    sg_4_raw: u32,
}

impl OzysMeasurementMessage {
    pub fn new(sg_1: Option<f32>, sg_2: Option<f32>, sg_3: Option<f32>, sg_4: Option<f32>) -> Self {
        Self {
            sg_1_raw: u32::from_be_bytes(sg_1.unwrap_or(f32::NAN).to_be_bytes()),
            sg_2_raw: u32::from_be_bytes(sg_2.unwrap_or(f32::NAN).to_be_bytes()),
            sg_3_raw: u32::from_be_bytes(sg_3.unwrap_or(f32::NAN).to_be_bytes()),
            sg_4_raw: u32::from_be_bytes(sg_4.unwrap_or(f32::NAN).to_be_bytes()),
        }
    }

    pub fn sg_1(&self) -> Option<f32> {
        match f32::from_bits(self.sg_1_raw) {
            v if v.is_nan() => None,
            v => Some(v),
        }
    }

    pub fn sg_2(&self) -> Option<f32> {
        match f32::from_bits(self.sg_2_raw) {
            v if v.is_nan() => None,
            v => Some(v),
        }
    }

    pub fn sg_3(&self) -> Option<f32> {
        match f32::from_bits(self.sg_3_raw) {
            v if v.is_nan() => None,
            v => Some(v),
        }
    }

    pub fn sg_4(&self) -> Option<f32> {
        match f32::from_bits(self.sg_4_raw) {
            v if v.is_nan() => None,
            v => Some(v),
        }
    }
}

impl CanBusMessage for OzysMeasurementMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for OzysMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::OzysMeasurement(self)
    }
}
