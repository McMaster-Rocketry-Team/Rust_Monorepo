use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{can_bus::custom_status::NodeCustomStatus};


#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct OzysCustomStatus {
    pub sg_1_connected: bool,
    pub sg_2_connected: bool,
    pub sg_3_connected: bool,
    pub sg_4_connected: bool,
    pub sd_ok: bool,

    #[packed_field(size_bits = "6")]
    disk_usage_raw: u8,
}

impl OzysCustomStatus {
    pub fn new(
        sg_1_connected: bool,
        sg_2_connected: bool,
        sg_3_connected: bool,
        sg_4_connected: bool,
        sd_ok: bool,
        disk_usage: f32, // 0.0 - 1.0
    ) -> Self {
        Self {
            sg_1_connected,
            sg_2_connected,
            sg_3_connected,
            sg_4_connected,
            sd_ok,
            disk_usage_raw: (disk_usage * 63.0) as u8,
        }
    }
    
    pub fn disk_usage(&self) -> f32 {
        self.disk_usage_raw as f32 / 63.0
    }
}


impl NodeCustomStatus for OzysCustomStatus {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_disk_usage_conversion() {
        // Test minimum value
        let status = OzysCustomStatus::new(false, false, false, false, false, 0.0);
        assert_eq!(status.disk_usage_raw, 0);
        assert_eq!(status.disk_usage(), 0.0);

        // Test maximum value
        let status = OzysCustomStatus::new(false, false, false, false, false, 1.0);
        assert_eq!(status.disk_usage_raw, 63);
        assert_eq!(status.disk_usage(), 1.0);

        // Test precision near boundaries
        let status = OzysCustomStatus::new(false, false, false, false, false, 0.999);
        assert_eq!(status.disk_usage_raw, 62);
        
        let status = OzysCustomStatus::new(false, false, false, false, false, 0.001);
        assert_eq!(status.disk_usage_raw, 0);
    }
}
