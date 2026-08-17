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
    /// Every flag starts false: a subsystem is unhealthy until it has proven
    /// otherwise.
    ///
    /// These are the values the node reports for the whole window between boot
    /// and each task's first success, and they are what it keeps reporting for
    /// any subsystem that never gets that far. Defaulting them true meant a
    /// sensor that was absent, unpowered or mis-wired — one that therefore
    /// never ran the code that would have cleared its flag — was indistinguishable
    /// on the wire from a healthy one, and the boot self-test read them
    /// straight into its pass/fail decision. Each owning task now raises its
    /// flag on first success and lowers it on failure.
    pub fn new() -> Self {
        Self {
            imu_ok: false,
            baro_ok: false,
            mag_ok: false,
            gps_ok: false,
            sd_ok: false,
            can_bus_ok: false,
        }
    }
}

impl NodeCustomStatus for VLCustomStatus {}

#[cfg(test)]
mod tests {
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
