use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
pub struct AckPacket {
    pub sha: u16,
}