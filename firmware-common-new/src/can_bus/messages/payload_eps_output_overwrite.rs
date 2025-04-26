use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputOverwrite {
    NoOverwrite = 0,
    ForceEnabled = 1,
    ForceDisabled = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct PayloadEPSOutputOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out_5v: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out_9v: PowerOutputOverwrite,

    /// Node ID of EPS to control
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl PayloadEPSOutputOverwriteMessage {
    pub fn new(
        out_3v3: PowerOutputOverwrite,
        out_5v: PowerOutputOverwrite,
        out_9v: PowerOutputOverwrite,
        node_id: u16,
    ) -> Self {
        Self {
            out_3v3,
            out_5v,
            out_9v,
            node_id,
        }
    }
}

impl CanBusMessage for PayloadEPSOutputOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}
