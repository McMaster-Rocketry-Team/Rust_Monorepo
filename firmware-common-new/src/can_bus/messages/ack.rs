use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct AckMessage {
    /// CRC of the message that was acknowledged
    pub crc: u16,

    /// Node ID of the sender
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl CanBusMessage for AckMessage {
    fn priority(&self) -> u8 {
        4
    }
}

impl Into<CanBusMessageEnum> for AckMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::Ack(self)
    }
}
