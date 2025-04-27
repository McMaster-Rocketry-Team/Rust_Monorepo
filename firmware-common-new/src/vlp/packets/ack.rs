use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::VLPDownlinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
pub struct AckPacket {
    pub sha: u16,
}

impl Into<VLPDownlinkPacket> for AckPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::Ack(self)
    }
}
