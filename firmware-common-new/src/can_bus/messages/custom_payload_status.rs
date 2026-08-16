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
    ///
    /// The struct fields stay raw `u16` because that is what goes on the wire
    /// and what a `PackedStruct` literal has to be built from, but nothing
    /// downstream should be reading them directly — every accessor below runs
    /// the field through here first, so a caller gets an `Option` it has to
    /// deal with rather than a `0xFFFF` it has to remember to check for. Each
    /// accessor is deliberately named exactly like its field, so
    /// `msg.epm_batt_mv()` is the obvious thing to reach for and
    /// `msg.epm_batt_mv` is the thing you have to go out of your way to write.
    pub fn reading(raw: u16) -> Option<u16> {
        if raw == PAYLOAD_READING_UNAVAILABLE {
            None
        } else {
            Some(raw)
        }
    }

    /// EPM battery bus voltage, mV. `None` if EPM could not read it.
    pub fn epm_batt_mv(&self) -> Option<u16> {
        Self::reading(self.epm_batt_mv)
    }

    /// System 3.3V rail load current, mA. `None` if EPM could not read it.
    /// A rail that is switched off reads `Some(0)`, not `None`.
    pub fn epm_sys_3v3_ma(&self) -> Option<u16> {
        Self::reading(self.epm_sys_3v3_ma)
    }

    /// System 5V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_sys_5v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_sys_5v_ma)
    }

    /// Peripheral 3.3V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_3v3_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_3v3_ma)
    }

    /// Peripheral 5V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_5v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_5v_ma)
    }

    /// Peripheral 9V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_9v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_9v_ma)
    }

    /// Peripheral 12V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_12v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_12v_ma)
    }

    /// Experiment channel 1 actuator position, steps. `None` if SEM could not
    /// read it. An actuator parked at its home position reads `Some(0)`.
    pub fn sem_actuator_1_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_1_steps)
    }

    /// Experiment channel 2 actuator position, steps. `None` if SEM could not
    /// read it.
    pub fn sem_actuator_2_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_2_steps)
    }

    /// Experiment channel 3 actuator position, steps. `None` if SEM could not
    /// read it.
    pub fn sem_actuator_3_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_3_steps)
    }

    /// The six rail currents in the stack's rail index order (0 `SYS_3V3`,
    /// 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`, 4 `PER_9V`, 5 `PER_12V`), each
    /// `None` if EPM could not read that rail. Rails fail to read
    /// individually — one dead INA does not take the other five with it — so
    /// this is an array of `Option`, not an `Option` of an array.
    pub fn rail_ma(&self) -> [Option<u16>; 6] {
        [
            self.epm_sys_3v3_ma(),
            self.epm_sys_5v_ma(),
            self.epm_per_3v3_ma(),
            self.epm_per_5v_ma(),
            self.epm_per_9v_ma(),
            self.epm_per_12v_ma(),
        ]
    }

    /// Actuator positions for experiment channels 1..3, each `None` if SEM
    /// could not read that channel.
    pub fn actuator_steps(&self) -> [Option<u16>; 3] {
        [
            self.sem_actuator_1_steps(),
            self.sem_actuator_2_steps(),
            self.sem_actuator_3_steps(),
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
        assert_eq!(message.epm_batt_mv(), Some(12600));
        assert_eq!(
            message.rail_ma(),
            [Some(1), Some(2), Some(3), Some(4), Some(5), Some(6)]
        );
        assert_eq!(message.actuator_steps(), [Some(7), Some(8), Some(9)]);
    }

    /// A reading that is unavailable has to stay unavailable all the way to
    /// the caller, and a genuine 0 has to survive as a 0 — a switched-off rail
    /// and an actuator at its home position both read 0 in normal operation.
    #[test]
    fn unavailable_readings_are_none_and_zeros_are_not() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        assert_eq!(message.epm_batt_mv(), None);
        assert_eq!(message.rail_ma(), [None; 6]);
        assert_eq!(message.actuator_steps(), [None; 3]);

        let message = CustomPayloadStatusMessage {
            epm_batt_mv: 0,
            epm_sys_3v3_ma: 0,
            epm_sys_5v_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_3v3_ma: 0,
            epm_per_5v_ma: 0,
            epm_per_9v_ma: 0,
            epm_per_12v_ma: 0,
            sem_actuator_1_steps: 0,
            sem_actuator_2_steps: PAYLOAD_READING_UNAVAILABLE,
            sem_actuator_3_steps: 0,
        };
        assert_eq!(message.epm_batt_mv(), Some(0));
        assert_eq!(
            message.rail_ma(),
            [Some(0), None, Some(0), Some(0), Some(0), Some(0)]
        );
        assert_eq!(message.actuator_steps(), [Some(0), None, Some(0)]);
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
