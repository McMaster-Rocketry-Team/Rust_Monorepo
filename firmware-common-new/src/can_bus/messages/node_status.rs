use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum NodeHealth {
    /// The node is functioning properly.
    Healthy = 0,
    /// A critical parameter went out of range or the node encountered a minor failure.
    Warning = 1,
    /// The node encountered a major failure.
    Error = 2,
    ///The node suffered a fatal malfunction.
    Critical = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum NodeMode {
    /// Normal operating mode.
    Operational = 0,
    /// Initialization is in progress; this mode is entered immediately after startup.
    Initialization = 1,
    /// E.g. calibration, the bootloader is running, etc.
    Maintainance = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
#[repr(C)]
pub struct NodeStatusMessage {
    pub uptime_s: u32,
    #[packed_field(bits = "32..36", ty = "enum")]
    pub health: NodeHealth,
    #[packed_field(bits = "36..40", ty = "enum")]
    pub mode: NodeMode,
}

impl NodeStatusMessage {
    pub fn new(
        uptime_s: u32,
        health: NodeHealth,
        mode: NodeMode,
    ) -> Self {
        Self {
            uptime_s,
            health,
            mode,
        }
    }
}

impl CanBusMessage for NodeStatusMessage {
    fn len() -> usize {
        5
    }

    fn priority(&self) -> u8 {
        5
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..Self::len()]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
