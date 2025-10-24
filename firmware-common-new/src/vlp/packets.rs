use core::fmt::Debug;

use crate::{utils::FixedLenSerializable};
use ack::AckPacket;
use amp_output_overwrite::AMPOutputOverwritePacket;
use change_mode::ChangeModePacket;
use fire_pyro::FirePyroPacket;
use set_target_apogee::SetTargetApogeePacket;
use gps_beacon::GPSBeaconPacket;
use low_power_telemetry::LowPowerTelemetryPacket;
use payload_eps_output_overwrite::PayloadEPSOutputOverwritePacket;
use reset::ResetPacket;
use self_test_result::SelfTestResultPacket;
use telemetry::TelemetryPacket;
use landed_telemetry::LandedTelemetryPacket;

pub mod ack;
pub mod amp_output_overwrite;
pub mod change_mode;
pub mod fire_pyro;
pub mod gps_beacon;
pub mod landed_telemetry;
pub mod low_power_telemetry;
pub mod payload_eps_output_overwrite;
pub mod reset;
pub mod self_test_result;
pub mod telemetry;
pub mod set_target_apogee;

// TODO change
pub const MAX_VLP_PACKET_SIZE: usize = 100;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VLPDownlinkPacket {
    GPSBeacon(GPSBeaconPacket),
    Ack(AckPacket),
    LowPowerTelemetry(LowPowerTelemetryPacket),
    Telemetry(TelemetryPacket),
    SelfTestResult(SelfTestResultPacket),
    LandedTelemetry(LandedTelemetryPacket),
}

impl VLPDownlinkPacket {
    pub fn deserialize(data: &[u8]) -> Option<Self> {
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
            4 => SelfTestResultPacket::deserialize(data).map(VLPDownlinkPacket::SelfTestResult),
            5 => LandedTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LandedTelemetry),
            _ => None,
        }
    }

    pub fn packet_type(&self) -> u8 {
        match self {
            VLPDownlinkPacket::GPSBeacon(_) => 0,
            VLPDownlinkPacket::Ack(_) => 1,
            VLPDownlinkPacket::LowPowerTelemetry(_) => 2,
            VLPDownlinkPacket::Telemetry(_) => 3,
            VLPDownlinkPacket::SelfTestResult(_) => 4,
            VLPDownlinkPacket::LandedTelemetry(_) => 5,
        }
    }

    pub fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = self.packet_type();
        buffer = &mut buffer[1..];

        1 + match self {
            VLPDownlinkPacket::GPSBeacon(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::Ack(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::LowPowerTelemetry(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::Telemetry(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::SelfTestResult(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::LandedTelemetry(packet) => packet.serialize(buffer),
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VLPUplinkPacket {
    ChangeMode(ChangeModePacket),
    Reset(ResetPacket),
    PayloadEPSOutputOverwrite(PayloadEPSOutputOverwritePacket),
    AMPOutputOverwrite(AMPOutputOverwritePacket),
    FirePyro(FirePyroPacket),
    SetTargetApogee(SetTargetApogeePacket)
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
            1 => ResetPacket::deserialize(data).map(VLPUplinkPacket::Reset),
            2 => PayloadEPSOutputOverwritePacket::deserialize(data)
                .map(VLPUplinkPacket::PayloadEPSOutputOverwrite),
            3 => {
                AMPOutputOverwritePacket::deserialize(data).map(VLPUplinkPacket::AMPOutputOverwrite)
            }
            4 => FirePyroPacket::deserialize(data).map(VLPUplinkPacket::FirePyro),
            5 => SetTargetApogeePacket::deserialize(data).map(VLPUplinkPacket::SetTargetApogee),
            _ => None,
        }
    }

    pub(super) fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPUplinkPacket::ChangeMode(_) => 0,
            VLPUplinkPacket::Reset(_) => 1,
            VLPUplinkPacket::PayloadEPSOutputOverwrite(_) => 2,
            VLPUplinkPacket::AMPOutputOverwrite(_) => 3,
            VLPUplinkPacket::FirePyro(_) => 4,
            VLPUplinkPacket::SetTargetApogee(_) => 5,
        };
        buffer = &mut buffer[1..];

        1 + match self {
            VLPUplinkPacket::ChangeMode(packet) => packet.serialize(buffer),
            VLPUplinkPacket::Reset(packet) => packet.serialize(buffer),
            VLPUplinkPacket::PayloadEPSOutputOverwrite(packet) => packet.serialize(buffer),
            VLPUplinkPacket::AMPOutputOverwrite(packet) => packet.serialize(buffer),
            VLPUplinkPacket::FirePyro(packet) => packet.serialize(buffer),
            VLPUplinkPacket::SetTargetApogee(packet) => packet.serialize(buffer),
        }
    }
}
