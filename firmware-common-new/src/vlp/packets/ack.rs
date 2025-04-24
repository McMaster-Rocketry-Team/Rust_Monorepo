use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "8")]
pub struct AckPacket {
    // we technically don't need so many bits for sha,
    // I put 8 bytes here just so we can leverage ecc.
    // which requires a > 7 bytes packet to be able to correct errors.
    pub sha: u64,
}