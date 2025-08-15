use core::cell::{RefCell, RefMut};

use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
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
    pub rebooted_in_last_5s: bool,

    /// Node specific status, only the lower 11 bits are used.
    #[packed_field(bits = "5..16")]
    pub custom_status: u16,
}

impl NodeStatus {
    pub fn healthy(&self) -> bool {
        self.health == NodeHealth::Healthy && self.mode == NodeMode::Operational
    }

    pub fn offline() -> Self {
        NodeStatus {
            health: NodeHealth::Error,
            mode: NodeMode::Offline,
            rebooted_in_last_5s: false,
            custom_status: 0,
        }
    }

    pub fn from_message(message: &NodeStatusMessage) -> Self {
        Self {
            health: message.health,
            mode: message.mode,
            rebooted_in_last_5s: message.uptime_s < 5,
            custom_status: message.custom_status_raw,
        }
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            health: format!("{:?}", self.health),
            mode: format!("{:?}", self.mode),
            rebooted_in_last_5s: format!("{:?}", self.rebooted_in_last_5s),
            custom_status: format!("0b{:012b}", self.custom_status),
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "22")]
pub struct SelfTestResultPacket {
    #[packed_field(bits = "0..4")]
    nonce: u8,

    #[packed_field(element_size_bytes = "2")]
    pub amp: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub icarus: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub ozys1: NodeStatus,

    #[packed_field(element_size_bytes = "2")]
    pub ozys2: NodeStatus,

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
    pub amp_out1_power_good: bool,
    pub amp_out2_power_good: bool,
    pub amp_out3_power_good: bool,
    pub amp_out4_power_good: bool,
    pub main_continuity: bool,
    pub drogue_continuity: bool,
}

impl SelfTestResultPacket {
    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            nonce: u8::from(self.nonce),
            imu_ok: self.imu_ok,
            baro_ok: self.baro_ok,
            mag_ok: self.mag_ok,
            gps_ok: self.gps_ok,
            sd_ok: self.sd_ok,
            can_bus_ok: self.can_bus_ok,
            amp_out1_power_good: self.amp_out1_power_good,
            amp_out2_power_good: self.amp_out2_power_good,
            amp_out3_power_good: self.amp_out3_power_good,
            amp_out4_power_good: self.amp_out4_power_good,
            main_continuity: self.main_continuity,
            drogue_continuity: self.drogue_continuity,

            amp: self.amp.to_json(),
            icarus: self.icarus.to_json(),
            ozys1: self.ozys1.to_json(),
            ozys2: self.ozys2.to_json(),
            payload_activation_pcb: self.payload_activation_pcb.to_json(),
            rocket_wifi: self.rocket_wifi.to_json(),
            payload_eps1: self.payload_eps1.to_json(),
            payload_eps2: self.payload_eps2.to_json(),
            main_bulkhead_pcb: self.main_bulkhead_pcb.to_json(),
            drogue_bulkhead_pcb: self.drogue_bulkhead_pcb.to_json(),
        }
    }
}

impl Into<VLPDownlinkPacket> for SelfTestResultPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::SelfTestResult(self)
    }
}

pub struct SelfTestResultPacketBuilderState {
    nonce: u8,
    pub amp: NodeStatus,
    pub icarus: NodeStatus,
    pub ozys1: NodeStatus,
    pub ozys2: NodeStatus,
    pub payload_activation_pcb: NodeStatus,
    pub rocket_wifi: NodeStatus,
    pub payload_eps1: NodeStatus,
    pub payload_eps2: NodeStatus,
    pub main_bulkhead_pcb: NodeStatus,
    pub drogue_bulkhead_pcb: NodeStatus,
    pub imu_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub sd_ok: bool,
    pub can_bus_ok: bool,
    pub amp_out1_power_good: bool,
    pub amp_out2_power_good: bool,
    pub amp_out3_power_good: bool,
    pub amp_out4_power_good: bool,
    pub main_continuity: bool,
    pub drogue_continuity: bool,
}

pub struct SelfTestResultPacketBuilder<M: RawMutex> {
    state: BlockingMutex<M, RefCell<SelfTestResultPacketBuilderState>>,
}

impl<M: RawMutex> SelfTestResultPacketBuilder<M> {
    pub fn new() -> Self {
        Self {
            state: BlockingMutex::new(RefCell::new(SelfTestResultPacketBuilderState {
                nonce: 0,
                amp: NodeStatus::offline(),
                icarus: NodeStatus::offline(),
                ozys1: NodeStatus::offline(),
                ozys2: NodeStatus::offline(),
                payload_activation_pcb: NodeStatus::offline(),
                rocket_wifi: NodeStatus::offline(),
                payload_eps1: NodeStatus::offline(),
                payload_eps2: NodeStatus::offline(),
                main_bulkhead_pcb: NodeStatus::offline(),
                drogue_bulkhead_pcb: NodeStatus::offline(),
                imu_ok: false,
                baro_ok: false,
                mag_ok: false,
                gps_ok: false,
                sd_ok: false,
                can_bus_ok: false,
                amp_out1_power_good: false,
                amp_out2_power_good: false,
                amp_out3_power_good: false,
                amp_out4_power_good: false,
                main_continuity: false,
                drogue_continuity: false,
            })),
        }
    }

    pub fn create_packet(&self) -> SelfTestResultPacket {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.nonce += 1;
            if state.nonce > 15 {
                state.nonce = 0;
            }
            SelfTestResultPacket {
                nonce: state.nonce.into(),
                amp: state.amp.clone(),
                icarus: state.icarus.clone(),
                ozys1: state.ozys1.clone(),
                ozys2: state.ozys2.clone(),
                payload_activation_pcb: state.payload_activation_pcb.clone(),
                rocket_wifi: state.rocket_wifi.clone(),
                payload_eps1: state.payload_eps1.clone(),
                payload_eps2: state.payload_eps2.clone(),
                main_bulkhead_pcb: state.main_bulkhead_pcb.clone(),
                drogue_bulkhead_pcb: state.drogue_bulkhead_pcb.clone(),
                imu_ok: state.imu_ok,
                baro_ok: state.baro_ok,
                mag_ok: state.mag_ok,
                gps_ok: state.gps_ok,
                sd_ok: state.sd_ok,
                can_bus_ok: state.can_bus_ok,
                amp_out1_power_good: state.amp_out1_power_good,
                amp_out2_power_good: state.amp_out2_power_good,
                amp_out3_power_good: state.amp_out3_power_good,
                amp_out4_power_good: state.amp_out4_power_good,
                main_continuity: state.main_continuity,
                drogue_continuity: state.drogue_continuity,
            }
        })
    }

    pub fn update<U>(&self, update_fn: U)
    where
        U: FnOnce(&mut RefMut<SelfTestResultPacketBuilderState>) -> (),
    {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            update_fn(&mut state);
        })
    }
}
