#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "wasm")]
use tsify::Tsify;

use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
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
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[repr(C)]
pub enum NodeMode {
    /// Normal operating mode.
    Operational = 0,
    /// Initialization is in progress; this mode is entered immediately after startup.
    Initialization = 1,
    /// E.g. calibration, the bootloader is running, etc.
    Maintainance = 2,
    /// Mode Offline can be reported by the node to explicitly inform other nodes in
    /// the network that it is shutting down.
    /// Additionally, this value is used for telemetry to tell the ground station that
    /// a node is offline.
    Offline = 3,
}

/// Every node in the network should send this message every 1s.
/// If a node does not send this message for 2s, it is considered offline.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
#[repr(C)]
pub struct NodeStatusMessage {
    #[packed_field(bits = "0..24")]
    pub uptime_s: u32,
    #[packed_field(bits = "24..26", ty = "enum")]
    pub health: NodeHealth,
    #[packed_field(bits = "26..28", ty = "enum")]
    pub mode: NodeMode,
    
    /// Node specific status, only the lower 12 bits are used.
    #[packed_field(bits = "28..40")]
    pub custom_status: u16,
}

impl CanBusMessage for NodeStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for NodeStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::NodeStatus(self)
    }
}
