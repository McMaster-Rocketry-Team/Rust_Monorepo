use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct ResetMessage {
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
    pub reset_all: bool,
}

impl ResetMessage {
    pub fn new(node_id: u16, reset_all: bool) -> Self {
        Self {
            node_id: node_id.into(),
            reset_all,
        }
    }
}

impl CanBusMessage for ResetMessage {
    fn priority(&self) -> u8 {
        0
    }
}
