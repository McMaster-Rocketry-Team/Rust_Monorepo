use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct AirBrakesControlMessage {
    /// Unit: 0.1%, e.g. 10 = 1%
    extension_percentage: u16,
}

impl AirBrakesControlMessage {
    /// percentage: 0 - 1
    pub fn new(
        extension_percentage: f32,
    ) -> Self {
        Self {
            extension_percentage: (extension_percentage * 1000.0) as u16,
        }
    }

    pub fn extension_percentage(&self) -> f32 {
        self.extension_percentage as f32 / 1000.0
    }
}

impl CanBusMessage for AirBrakesControlMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AirBrakesControlMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AirBrakesControl(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            AirBrakesControlMessage::new(0.0).into(),
            AirBrakesControlMessage::new(65.535).into(),
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "airbrakes_control");
    }
}
