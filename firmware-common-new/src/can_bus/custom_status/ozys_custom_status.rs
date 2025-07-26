use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::custom_status::NodeCustomStatus;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct OzysCustomStatus {
    pub sd_ok: bool,
    pub disk_usage: u8,
}

impl NodeCustomStatus for OzysCustomStatus {}

