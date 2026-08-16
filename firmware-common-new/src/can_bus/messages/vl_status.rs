use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

/// may skip stages, may go back to a previous stage
///
/// `LowPower` / `SelfTest` / `Armed` are device modes; the remaining values
/// mirror the deployment estimator's `RocketState` variants 1:1 — nothing is
/// folded (`MachLockout` and `FailedToReachMinApogee` report as themselves).
/// The `coasting` burn-timer flag and the chutes' `deployed` bools are
/// orthogonal to the stage and travel as separate bools next to it
/// (`RocketStateMessage::is_coasting` on CAN, dedicated bools in the VLP
/// telemetry packet, `coasting` / pyro fire flags in the flight data records).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[rkyv(derive(Clone, Copy, Debug))]
#[repr(u8)]
pub enum FlightStage {
    LowPower = 0,
    SelfTest = 1,
    /// Armed, still on the pad (`RocketState::OnPad`).
    Armed = 2,
    Ascent = 3,
    MachLockout = 4,
    DrogueChute = 5,
    MainChute = 6,
    Landed = 7,
    FailedToReachMinApogee = 8,
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
                flight_stage: FlightStage::MachLockout,
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
