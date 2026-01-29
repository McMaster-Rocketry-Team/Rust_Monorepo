use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct ResetMessage {
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
    pub reset_all: bool,
    pub into_bootloader: bool,
}

impl CanBusMessage for ResetMessage {
    fn priority(&self) -> u8 {
        0
    }
}

impl Into<CanBusMessageEnum> for ResetMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::Reset(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            ResetMessage {
                node_id: 0,
                reset_all: false,
                into_bootloader: false,
            }
            .into(),
            ResetMessage {
                node_id: 0xFFF,
                reset_all: true,
                into_bootloader: true,
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "reset");
    }
}
