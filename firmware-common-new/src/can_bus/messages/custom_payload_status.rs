use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

/// Reported for a reading that is invalid or unavailable.
pub const PAYLOAD_READING_UNAVAILABLE: u16 = 0xFFFF;

/// Extended EPM / SEM telemetry from the payload SDRM node, sent every 500ms.
///
/// Supplementary to `NodeStatusMessage`, which stays the primary go/no-go source.
/// Deliberately does not repeat `uptime_s`, `health`, `mode` or the stack flags, so
/// the two messages can not drift apart.
///
/// Everything here is relayed from EPM / SEM on the intra-stack bus, not measured
/// by SDRM: EPM reports the battery bus voltage and the load current of all six
/// switched rails, SEM reports the linear actuator positions.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "20")]
#[repr(C)]
pub struct CustomPayloadStatusMessage {
    /// EPM battery bus voltage
    pub epm_batt_mv: u16,

    /// System 3.3V rail load current
    pub epm_sys_3v3_ma: u16,
    /// System 5V rail load current
    pub epm_sys_5v_ma: u16,
    /// Peripheral 3.3V rail load current
    pub epm_per_3v3_ma: u16,
    /// Peripheral 5V rail load current
    pub epm_per_5v_ma: u16,
    /// Peripheral 9V rail load current
    pub epm_per_9v_ma: u16,
    /// Peripheral 12V rail load current
    pub epm_per_12v_ma: u16,

    /// SEM linear actuator position, experiment channel 1
    pub sem_actuator_1_steps: u16,
    /// SEM linear actuator position, experiment channel 2
    pub sem_actuator_2_steps: u16,
    /// SEM linear actuator position, experiment channel 3
    pub sem_actuator_3_steps: u16,
}

impl CustomPayloadStatusMessage {
    /// Every reading unavailable, e.g. before EPM / SEM have reported.
    pub fn new_unavailable() -> Self {
        Self {
            epm_batt_mv: PAYLOAD_READING_UNAVAILABLE,
            epm_sys_3v3_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_sys_5v_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_3v3_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_5v_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_9v_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_12v_ma: PAYLOAD_READING_UNAVAILABLE,
            sem_actuator_1_steps: PAYLOAD_READING_UNAVAILABLE,
            sem_actuator_2_steps: PAYLOAD_READING_UNAVAILABLE,
            sem_actuator_3_steps: PAYLOAD_READING_UNAVAILABLE,
        }
    }

    /// `None` if the reading is invalid or unavailable.
    pub fn reading(raw: u16) -> Option<u16> {
        if raw == PAYLOAD_READING_UNAVAILABLE {
            None
        } else {
            Some(raw)
        }
    }

    /// The six rail currents in the stack's rail index order (0 `SYS_3V3`,
    /// 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`, 4 `PER_9V`, 5 `PER_12V`).
    pub fn rail_ma(&self) -> [u16; 6] {
        [
            self.epm_sys_3v3_ma,
            self.epm_sys_5v_ma,
            self.epm_per_3v3_ma,
            self.epm_per_5v_ma,
            self.epm_per_9v_ma,
            self.epm_per_12v_ma,
        ]
    }

    /// Actuator positions for experiment channels 1..3.
    pub fn actuator_steps(&self) -> [u16; 3] {
        [
            self.sem_actuator_1_steps,
            self.sem_actuator_2_steps,
            self.sem_actuator_3_steps,
        ]
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
                epm_sys_3v3_ma: 0,
                epm_sys_5v_ma: 0,
                epm_per_3v3_ma: 0,
                epm_per_5v_ma: 0,
                epm_per_9v_ma: 0,
                epm_per_12v_ma: 0,
                sem_actuator_1_steps: 0,
                sem_actuator_2_steps: 0,
                sem_actuator_3_steps: 0,
            }
            .into(),
            CustomPayloadStatusMessage::new_unavailable().into(),
            CustomPayloadStatusMessage {
                epm_batt_mv: 12600,
                epm_sys_3v3_ma: 120,
                epm_sys_5v_ma: 340,
                epm_per_3v3_ma: 55,
                epm_per_5v_ma: 780,
                epm_per_9v_ma: 1500,
                epm_per_12v_ma: 2400,
                sem_actuator_1_steps: 0,
                sem_actuator_2_steps: 1200,
                sem_actuator_3_steps: 34567,
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
    fn test_reading() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        assert_eq!(
            CustomPayloadStatusMessage::reading(message.epm_batt_mv),
            None
        );
        assert_eq!(CustomPayloadStatusMessage::reading(0), Some(0));
        assert_eq!(CustomPayloadStatusMessage::reading(12600), Some(12600));
    }

    #[test]
    fn test_accessors() {
        let message = CustomPayloadStatusMessage {
            epm_batt_mv: 12600,
            epm_sys_3v3_ma: 1,
            epm_sys_5v_ma: 2,
            epm_per_3v3_ma: 3,
            epm_per_5v_ma: 4,
            epm_per_9v_ma: 5,
            epm_per_12v_ma: 6,
            sem_actuator_1_steps: 7,
            sem_actuator_2_steps: 8,
            sem_actuator_3_steps: 9,
        };
        assert_eq!(message.rail_ma(), [1, 2, 3, 4, 5, 6]);
        assert_eq!(message.actuator_steps(), [7, 8, 9]);
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
