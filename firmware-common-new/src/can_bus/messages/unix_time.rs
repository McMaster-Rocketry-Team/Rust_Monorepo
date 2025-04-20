use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
pub struct UnixTimeMessage {
    /// Current milliseconds since Unix epoch, floored to the nearest ms
    pub timestamp: Integer<u64, packed_bits::Bits<48>>,
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

    fn priority() -> u8 {
        6
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
