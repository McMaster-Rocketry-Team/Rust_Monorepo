use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputOverwrite {
    NoOverwrite = 0,
    ForceEnabled = 1,
    ForceDisabled = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out1: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out2: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out3: PowerOutputOverwrite,
    #[packed_field(bits = "6..8", ty = "enum")]
    pub out4: PowerOutputOverwrite,
}

impl CanBusMessage for AmpOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AmpOverwriteMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpOverwrite(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            AmpOverwriteMessage {
                out1: PowerOutputOverwrite::NoOverwrite,
                out2: PowerOutputOverwrite::ForceEnabled,
                out3: PowerOutputOverwrite::ForceDisabled,
                out4: PowerOutputOverwrite::NoOverwrite,
            }
            .into(),
            AmpOverwriteMessage {
                out1: PowerOutputOverwrite::ForceEnabled,
                out2: PowerOutputOverwrite::ForceDisabled,
                out3: PowerOutputOverwrite::NoOverwrite,
                out4: PowerOutputOverwrite::ForceEnabled,
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "amp_overwrite");
    }
}
