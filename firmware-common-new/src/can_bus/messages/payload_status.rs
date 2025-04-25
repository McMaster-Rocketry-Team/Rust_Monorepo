use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{amp_status::PowerOutputStatus, CanBusMessage};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct EPSOutputStatus {
    #[packed_field(bits = "0..14")]
    pub current_ma: u16,
    #[packed_field(bits = "14..16", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
#[repr(C)]
pub struct EPSStatus {
    pub battery1_mv: u16,
    pub battery2_mv: u16,

    #[packed_field(element_size_bytes = "2")]
    pub output_3v3: EPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_5v: EPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_9v: EPSOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "25")]
#[repr(C)]
pub struct PayloadStatusMessage {
    #[packed_field(element_size_bytes = "10")]
    pub eps1: EPSStatus,
    #[packed_field(element_size_bytes = "10")]
    pub eps2: EPSStatus,

    #[packed_field(element_size_bits = "12")]
    pub eps1_node_id: u16,
    #[packed_field(element_size_bits = "12")]
    pub eps2_node_id: u16,
    #[packed_field(element_size_bits = "12")]
    pub rocket_wifi_node_id: u16,

    #[packed_field(element_size_bits = "4")]
    _padding: u8,
}

impl PayloadStatusMessage {
    pub fn new(
        eps1: EPSStatus,
        eps2: EPSStatus,
        eps1_node_id: u16,
        eps2_node_id: u16,
        rocket_wifi_node_id: u16,
    ) -> Self {
        Self {
            eps1,
            eps2,
            eps1_node_id: eps1_node_id.into(),
            eps2_node_id: eps2_node_id.into(),
            rocket_wifi_node_id: rocket_wifi_node_id.into(),
            _padding: Default::default(),
        }
    }
}

impl CanBusMessage for PayloadStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}
