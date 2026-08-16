use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[rkyv(derive(Clone, Copy, Debug))]
#[repr(u8)]
pub enum FlightStage {
    LowPower = 0,
    SelfTest = 1,
    /// Armed, still on the pad (`RocketState::OnPad`).
    Armed = 2,
    /// Ascending, powered or coasting.
    Ascent = 3,
    DrogueChute = 4,
    MainChute = 5,
    Landed = 6,
    FailedToReachMinApogee = 7,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
#[repr(C)]
pub struct VLStatusMessage {
    #[packed_field(bits = "0..8", ty = "enum")]
    pub flight_stage: FlightStage,
    pub battery_mv: u16,
}

impl CanBusMessage for VLStatusMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for VLStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::VLStatus(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            VLStatusMessage {
                flight_stage: FlightStage::LowPower,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::Landed,
                battery_mv: u16::MAX,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::SelfTest,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::Armed,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::Ascent,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::DrogueChute,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::MainChute,
                battery_mv: 0,
            }
            .into(),
            VLStatusMessage {
                flight_stage: FlightStage::FailedToReachMinApogee,
                battery_mv: 0,
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "vl_status");
    }
}
