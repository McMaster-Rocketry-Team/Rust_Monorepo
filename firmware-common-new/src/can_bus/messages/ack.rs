use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct AckMessage {
    /// CRC of the message that was acknowledged
    pub crc: u16,

    /// Node ID of the sender
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,

    #[packed_field(element_size_bits = "4")]
    _padding: u8,
}

impl AckMessage {
    pub fn new(crc: u16, node_id: u16) -> Self {
        Self {
            crc,
            node_id: node_id.into(),
            _padding: Default::default(),
        }
    }
}

impl CanBusMessage for AckMessage {
    fn len() -> usize {
        4
    }

    fn priority(&self) -> u8 {
        1
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
