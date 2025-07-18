use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

/// may skip stages, may go back to a previous stage
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[repr(C)]
pub enum FlightStage {
    LowPower = 0,
    SelfTest = 1,
    Armed = 2,
    PoweredAscent = 3,
    Coasting = 4,
    DrogueDeployed = 5,
    MainDeployed = 6,
    Landed = 7,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AvionicsStatusMessage {
    #[packed_field(bits = "0..8", ty = "enum")]
    pub flight_stage: FlightStage,
}

impl CanBusMessage for AvionicsStatusMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for AvionicsStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AvionicsStatus(self)
    }
}
