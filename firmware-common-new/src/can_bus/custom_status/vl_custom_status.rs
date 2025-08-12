use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::custom_status::NodeCustomStatus;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct VLCustomStatus {
    pub imu_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub sd_ok: bool,
    pub can_bus_ok: bool,
}

impl VLCustomStatus {
    pub fn new() -> Self {
        Self {
            imu_ok: true,
            baro_ok: true,
            mag_ok: true,
            gps_ok: true,
            sd_ok: true,
            can_bus_ok: true,
        }
    }
}

impl NodeCustomStatus for VLCustomStatus {}

#[cfg(test)]
mod test {
    use crate::can_bus::custom_status::NodeCustomStatusExt;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let status = VLCustomStatus {
            imu_ok: true,
            baro_ok: false,
            mag_ok: false,
            gps_ok: false,
            sd_ok: false,
            can_bus_ok: true,
        };

        let status_u16 = status.to_u16();
        assert_eq!(status_u16, 0b10000100000);

        let status2 = VLCustomStatus::from_u16(status_u16);
        assert_eq!(status, status2);
    }
}
