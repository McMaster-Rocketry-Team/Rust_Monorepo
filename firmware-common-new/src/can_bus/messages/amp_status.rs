use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(
    PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize,
)]
#[repr(C)]
pub enum PowerOutputStatus {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpOutputStatus {
    #[packed_field(bits = "0..1")]
    pub overwrote: bool,
    #[packed_field(bits = "1..3", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct AmpStatusMessage {
    pub shared_battery_mv: u16,

    // Can't use `#[packed_field(element_size_bits = "3")]` here due to packed_struct crate bug
    #[packed_field(element_size_bytes = "1")]
    pub out1: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out2: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out3: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out4: AmpOutputStatus,
}

impl CanBusMessage for AmpStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for AmpStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpStatus(self)
    }
}

#[cfg(test)]
mod tests {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};

    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            AmpStatusMessage {
                shared_battery_mv: 0,
                out1: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerGood,
                },
                out2: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Disabled,
                },
                out3: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerBad,
                },
                out4: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::PowerBad,
                },
            }
            .into(),
            AmpStatusMessage {
                shared_battery_mv: u16::MAX,
                out1: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerGood,
                },
                out2: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Disabled,
                },
                out3: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerBad,
                },
                out4: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::PowerBad,
                },
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

        can_bus_messages_test::create_reference_data(create_test_messages(), "amp_status");
    }
}
