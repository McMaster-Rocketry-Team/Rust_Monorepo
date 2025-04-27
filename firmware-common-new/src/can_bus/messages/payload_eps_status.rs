use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{amp_status::PowerOutputStatus, CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct PayloadEPSOutputStatus {
    #[packed_field(bits = "0..13")]
    pub current_ma: u16,
    #[packed_field(bits = "13..14")]
    pub overwrote: bool,
    #[packed_field(bits = "14..16", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "14")]
#[repr(C)]
pub struct PayloadEPSStatusMessage {
    pub battery1_mv: u16,
    /// Unit: 0.1C, e.g. 250 = 25C
    pub battery1_temperature: u16,

    pub battery2_mv: u16,
    /// Unit: 0.1C, e.g. 250 = 25C
    pub battery2_temperature: u16,

    #[packed_field(element_size_bytes = "2")]
    pub output_3v3: PayloadEPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_5v: PayloadEPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_9v: PayloadEPSOutputStatus,
}

impl CanBusMessage for PayloadEPSStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSStatus(self)
    }
}
