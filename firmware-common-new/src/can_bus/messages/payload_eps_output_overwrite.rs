use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum, amp_overwrite::PowerOutputOverwrite};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct PayloadEPSOutputOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out_5v: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out_9v: PowerOutputOverwrite,

    /// Node ID of EPS to control
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl CanBusMessage for PayloadEPSOutputOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSOutputOverwriteMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSOutputOverwrite(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            PayloadEPSOutputOverwriteMessage {
                out_3v3: PowerOutputOverwrite::NoOverwrite,
                out_5v: PowerOutputOverwrite::NoOverwrite,
                out_9v: PowerOutputOverwrite::NoOverwrite,
                node_id: 0,
            }
            .into(),
            PayloadEPSOutputOverwriteMessage {
                out_3v3: PowerOutputOverwrite::ForceEnabled,
                out_5v: PowerOutputOverwrite::ForceDisabled,
                out_9v: PowerOutputOverwrite::ForceEnabled,
                node_id: 0xFFF,
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "payload_eps_output_overwrite");
    }
}
