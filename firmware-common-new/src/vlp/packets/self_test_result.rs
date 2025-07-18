use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::messages::node_status::{NodeHealth, NodeMode, NodeStatusMessage};

use super::VLPDownlinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
pub struct NodeStatus {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub health: NodeHealth,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub mode: NodeMode,
    
    /// Node specific status, only the lower 12 bits are used.
    #[packed_field(bits = "4..16")]
    pub custom_status: u16,
}

impl NodeStatus {
    pub fn offline() -> Self {
        NodeStatus {
            health: NodeHealth::Error,
            mode: NodeMode::Offline,
            custom_status: 0,
        }
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            health: format!("{:?}", self.health),
            mode: format!("{:?}", self.mode),
            custom_status: format!("0b{:012b}", self.custom_status),
        }
    }
}

impl From<NodeStatusMessage> for NodeStatus {
    fn from(node_status_message: NodeStatusMessage) -> Self {
        NodeStatus {
            health: node_status_message.health,
            mode: node_status_message.mode,
            custom_status: node_status_message.custom_status,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "24")]
pub struct SelfTestResultPacket {
    #[packed_field(element_size_bytes = "2")]
    pub amp: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub icarus: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub ozys1: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub ozys2: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub aero_rust: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub payload_activation_pcb: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub rocket_wifi: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub payload_eps1: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub payload_eps2: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub main_bulkhead_pcb: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub drogue_bulkhead_pcb: NodeStatus,

    pub imu_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub sd_ok: bool,
    pub can_bus_ok: bool,

    pub amp_out1_ok: bool,
    pub amp_out2_ok: bool,
    pub amp_out3_ok: bool,
    pub amp_out4_ok: bool,
}

impl SelfTestResultPacket {
    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            imu_ok: self.imu_ok,
            baro_ok: self.baro_ok,
            mag_ok: self.mag_ok,
            gps_ok: self.gps_ok,
            sd_ok: self.sd_ok,
            can_bus_ok: self.can_bus_ok,

            amp: self.amp.to_json(),
            icarus: self.icarus.to_json(),
            ozys1: self.ozys1.to_json(),
            ozys2: self.ozys2.to_json(),
            aero_rust: self.aero_rust.to_json(),
            payload_activation_pcb: self.payload_activation_pcb.to_json(),
            rocket_wifi: self.rocket_wifi.to_json(),
            payload_eps1: self.payload_eps1.to_json(),
            payload_eps2: self.payload_eps2.to_json(),
            main_bulkhead_pcb: self.main_bulkhead_pcb.to_json(),
            drogue_bulkhead_pcb: self.drogue_bulkhead_pcb.to_json(),

            amp_out1_ok: self.amp_out1_ok,
            amp_out2_ok: self.amp_out2_ok,
            amp_out3_ok: self.amp_out3_ok,
            amp_out4_ok: self.amp_out4_ok,
        }
    }
}

impl Into<VLPDownlinkPacket> for SelfTestResultPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::SelfTestResult(self)
    }
}

