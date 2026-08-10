use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

/// Reported for a rail whose reading is invalid or unavailable.
pub const RAIL_MV_UNAVAILABLE: u16 = 0xFFFF;

/// Extended EPM telemetry from the payload SDRM node, sent every 500ms.
///
/// Supplementary to `NodeStatusMessage`, which stays the primary go/no-go source.
/// Deliberately does not repeat `uptime_s`, `health`, `mode` or the stack flags, so
/// the two messages can not drift apart.
///
/// Voltages are relayed from EPM on the intra-stack bus, not measured by SDRM.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
#[repr(C)]
pub struct CustomPayloadStatusMessage {
    /// EPM battery bus voltage
    pub epm_batt_mv: u16,
    /// EPM system 3.3V rail
    pub epm_sys_3v3_mv: u16,
    /// EPM system 5V rail
    pub epm_sys_5v_mv: u16,
    /// EPM peripheral 5V rail
    pub epm_per_5v_mv: u16,
    /// EPM peripheral 9V rail
    pub epm_per_9v_mv: u16,
}

impl CustomPayloadStatusMessage {
    /// Every rail unavailable, e.g. before EPM has reported.
    pub fn new_unavailable() -> Self {
        Self {
            epm_batt_mv: RAIL_MV_UNAVAILABLE,
            epm_sys_3v3_mv: RAIL_MV_UNAVAILABLE,
            epm_sys_5v_mv: RAIL_MV_UNAVAILABLE,
            epm_per_5v_mv: RAIL_MV_UNAVAILABLE,
            epm_per_9v_mv: RAIL_MV_UNAVAILABLE,
        }
    }

    /// `None` if the reading is invalid or unavailable.
    pub fn rail_mv(raw_mv: u16) -> Option<u16> {
        if raw_mv == RAIL_MV_UNAVAILABLE {
            None
        } else {
            Some(raw_mv)
        }
    }
}

impl CanBusMessage for CustomPayloadStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for CustomPayloadStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::CustomPayloadStatus(self)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            CustomPayloadStatusMessage {
                epm_batt_mv: 0,
                epm_sys_3v3_mv: 0,
                epm_sys_5v_mv: 0,
                epm_per_5v_mv: 0,
                epm_per_9v_mv: 0,
            }
            .into(),
            CustomPayloadStatusMessage::new_unavailable().into(),
            CustomPayloadStatusMessage {
                epm_batt_mv: 12600,
                epm_sys_3v3_mv: 3300,
                epm_sys_5v_mv: 5000,
                epm_per_5v_mv: 5000,
                epm_per_9v_mv: 9000,
            }
            .into(),
        ]
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();
        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn test_rail_mv() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        assert_eq!(
            CustomPayloadStatusMessage::rail_mv(message.epm_batt_mv),
            None
        );
        assert_eq!(CustomPayloadStatusMessage::rail_mv(0), Some(0));
        assert_eq!(CustomPayloadStatusMessage::rail_mv(12600), Some(12600));
    }

    #[test]
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(
            create_test_messages(),
            "custom_payload_status",
        );
    }
}
