use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct IcarusStatusMessage {
    /// Unit: 0.1%, e.g. 10 = 1%
    actual_extension_percentage: u16,
    /// Unit: 0.1C, e.g. 10 = 1C
    servo_temperature_raw: u16,
    /// Unit: 0.01A, e.g. 10 = 0.1A
    servo_current_raw: u16,
}

impl IcarusStatusMessage {
    /// percentage: 0 - 1
    pub fn new(
        actual_extension_percentage: f32,
        servo_temperature: f32,
        servo_current: f32,
    ) -> Self {
        Self {
            actual_extension_percentage: (actual_extension_percentage * 1000.0) as u16,
            servo_temperature_raw: (servo_temperature * 10.0) as u16,
            servo_current_raw: (servo_current * 100.0) as u16,
        }
    }

    pub fn actual_extension_percentage(&self) -> f32 {
        self.actual_extension_percentage as f32 / 1000.0
    }

    pub fn servo_temperature(&self) -> f32 {
        self.servo_temperature_raw as f32 / 10.0
    }

    pub fn servo_current(&self) -> f32 {
        self.servo_current_raw as f32 / 100.0
    }
}

impl CanBusMessage for IcarusStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for IcarusStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::IcarusStatus(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            IcarusStatusMessage::new(0.0, 0.0, 0.0).into(),
            IcarusStatusMessage::new(65.535, 6553.5, 655.35).into(),
        ]
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();
        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(create_test_messages(), "icarus_status");
    }
}
