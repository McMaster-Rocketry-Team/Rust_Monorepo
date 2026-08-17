use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct IcarusStatusMessage {
    /// Unit: 0.1%, e.g. 10 = 1%
    actual_extension_percentage: u16,
    /// Unit: 0.1C, e.g. 10 = 1C, -155 = -15.5C
    ///
    /// Signed, for the same reason as `BaroMeasurementMessage`: float-to-int
    /// `as` saturates, so an unsigned raw field reported every sub-zero servo
    /// as exactly 0.0C. A servo sitting on a cold pad is precisely the reading
    /// this field exists to show. `get_builtin_type_bit_width` maps `u16` and
    /// `i16` alike to 16, so the packed message is still 4 bytes.
    servo_temperature_raw: i16,
}

impl IcarusStatusMessage {
    /// percentage: 0 - 1
    pub fn new(actual_extension_percentage: f32, servo_temperature: f32) -> Self {
        Self {
            actual_extension_percentage: (actual_extension_percentage * 1000.0) as u16,
            servo_temperature_raw: (servo_temperature * 10.0) as i16,
        }
    }

    pub fn actual_extension_percentage(&self) -> f32 {
        self.actual_extension_percentage as f32 / 1000.0
    }

    pub fn servo_temperature(&self) -> f32 {
        self.servo_temperature_raw as f32 / 10.0
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
            IcarusStatusMessage::new(0.0, 0.0).into(),
            IcarusStatusMessage::new(65.535, 3276.7).into(),
            // A servo on a cold pad. The raw field used to be unsigned, which
            // reported this as 0.0C.
            IcarusStatusMessage::new(0.0, -15.5).into(),
        ]
    }

    /// Sub-zero servo temperatures used to saturate to 0 on the way in.
    #[test]
    fn sub_zero_temperatures_survive() {
        init_logger();

        assert_eq!(IcarusStatusMessage::new(0.0, -15.5).servo_temperature(), -15.5);
        assert_eq!(IcarusStatusMessage::new(0.0, -40.0).servo_temperature(), -40.0);
        assert_eq!(IcarusStatusMessage::new(0.0, 0.0).servo_temperature(), 0.0);
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
