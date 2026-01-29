use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::custom_status::NodeCustomStatusExt;

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(
    PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize,
)]
#[repr(C)]
pub enum NodeHealth {
    /// The node is functioning properly.
    Healthy = 0,
    /// A critical parameter went out of range or the node encountered a minor failure.
    Warning = 1,
    /// The node encountered a major failure.
    Error = 2,
    /// The node suffered a fatal malfunction.
    Critical = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(
    PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize,
)]
#[repr(C)]
pub enum NodeMode {
    /// Normal operating mode.
    Operational = 0,
    /// Initialization is in progress; this mode is entered immediately after startup.
    Initialization = 1,
    /// E.g. calibration, the bootloader is running, etc.
    Maintenance = 2,
    /// Mode Offline can be reported by the node to explicitly inform other nodes in
    /// the network that it is shutting down.
    /// Additionally, this value is used for telemetry to tell the ground station that
    /// a node is offline.
    Offline = 3,
}

/// Every node in the network should send this message every 1s.
/// If a node does not send this message for 2s, it is considered offline.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// Node specific status, only the lower 11 bits are used.
    #[packed_field(bits = "28..39")]
    pub custom_status_raw: u16,
}

impl NodeStatusMessage {
    pub fn new(
        uptime_s: u32,
        health: NodeHealth,
        mode: NodeMode,
        status: impl NodeCustomStatusExt,
    ) -> Self {
        Self {
            uptime_s,
            health,
            mode,
            custom_status_raw: status.to_u16(),
        }
    }

    pub fn new_no_custom_status(uptime_s: u32, health: NodeHealth, mode: NodeMode) -> Self {
        Self {
            uptime_s,
            health,
            mode,
            custom_status_raw: 0,
        }
    }

    pub fn custom_status<T: NodeCustomStatusExt>(&self) -> T {
        T::from_u16(self.custom_status_raw)
    }
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

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            NodeStatusMessage {
                uptime_s: 0,
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                custom_status_raw: 0,
            }
            .into(),
            NodeStatusMessage {
                uptime_s: 0xFFFFFF,
                health: NodeHealth::Critical,
                mode: NodeMode::Offline,
                custom_status_raw: 0x7FF,
            }
            .into(),
            NodeStatusMessage {
                uptime_s: 12345,
                health: NodeHealth::Warning,
                mode: NodeMode::Initialization,
                custom_status_raw: 0,
            }
            .into(),
            NodeStatusMessage {
                uptime_s: 67890,
                health: NodeHealth::Error,
                mode: NodeMode::Maintenance,
                custom_status_raw: 0,
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "node_status");
    }
}
