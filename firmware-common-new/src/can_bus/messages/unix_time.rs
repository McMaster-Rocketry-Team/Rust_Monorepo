use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct UnixTimeMessage {
    /// Current milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(element_size_bits = "48")]
    pub timestamp: u64,
}

impl UnixTimeMessage {
    pub fn new(timestamp: f64) -> Self {
        Self {
            timestamp: (timestamp as u64).into(),
        }
    }
}

impl CanBusMessage for UnixTimeMessage {
    fn len() -> usize {
        6
    }

    fn priority(&self) -> u8 {
        1
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..Self::len()]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
