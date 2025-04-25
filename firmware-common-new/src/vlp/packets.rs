use core::fmt::Debug;

use crate::utils::FixedLenSerializable;
use ack::AckPacket;
use change_mode::ChangeModePacket;
use gps_beacon::GPSBeaconPacket;
use low_power_telemetry::LowPowerTelemetryPacket;
use telemetry_packet::TelemetryPacket;

pub mod ack;
pub mod change_mode;
pub mod gps_beacon;
pub mod low_power_telemetry;
pub mod self_test_result_packet;
pub mod telemetry_packet;

// TODO change
pub const MAX_VLP_PACKET_SIZE: usize = 100;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum VLPDownlinkPacket {
    GPSBeacon(GPSBeaconPacket),
    Ack(AckPacket),
    LowPowerTelemetry(LowPowerTelemetryPacket),
    Telemetry(TelemetryPacket),
}

impl VLPDownlinkPacket {
    pub(super) fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => GPSBeaconPacket::deserialize(data).map(VLPDownlinkPacket::GPSBeacon),
            1 => AckPacket::deserialize(data).map(VLPDownlinkPacket::Ack),
            2 => {
                LowPowerTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LowPowerTelemetry)
            }
            3 => TelemetryPacket::deserialize(data).map(VLPDownlinkPacket::Telemetry),
            _ => None,
        }
    }

    pub(super) fn serialize(self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPDownlinkPacket::GPSBeacon(_) => 0,
            VLPDownlinkPacket::Ack(_) => 1,
            VLPDownlinkPacket::LowPowerTelemetry(_) => 2,
            VLPDownlinkPacket::Telemetry(_) => 3,
        };
        buffer = &mut buffer[1..];

        1 + match self {
            VLPDownlinkPacket::GPSBeacon(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::Ack(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::LowPowerTelemetry(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::Telemetry(packet) => packet.serialize(buffer),
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum VLPUplinkPacket {
    ChangeMode(ChangeModePacket),
}

impl VLPUplinkPacket {
    pub(super) fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => ChangeModePacket::deserialize(data).map(VLPUplinkPacket::ChangeMode),
            _ => None,
        }
    }

    pub(super) fn serialize(self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPUplinkPacket::ChangeMode(_) => 0,
        };
        buffer = &mut buffer[1..];

        1 + match self {
            VLPUplinkPacket::ChangeMode(packet) => packet.serialize(buffer),
        }
    }
}
