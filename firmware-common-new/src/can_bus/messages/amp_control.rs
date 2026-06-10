use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpControlMessage {
    pub out1_enable: bool,
    pub out2_enable: bool,
    pub out3_enable: bool,
    pub out4_enable: bool,
}

impl CanBusMessage for AmpControlMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AmpControlMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpControl(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            AmpControlMessage {
                out1_enable: false,
                out2_enable: false,
                out3_enable: false,
                out4_enable: false,
            }
            .into(),
            AmpControlMessage {
                out1_enable: true,
                out2_enable: true,
                out3_enable: true,
                out4_enable: true,
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
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(create_test_messages(), "amp_control");
    }
}
