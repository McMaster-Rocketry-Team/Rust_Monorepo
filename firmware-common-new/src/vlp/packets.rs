use core::fmt::Debug;

use crate::{fixed_point_factory, utils::FixedLenSerializable};
use ack::AckPacket;
use amp_output_overwrite::AMPOutputOverwritePacket;
use change_mode::ChangeModePacket;
use fire_pyro::FirePyroPacket;
use set_target_apogee::SetTargetApogeePacket;
use low_power_telemetry::LowPowerTelemetryPacket;
use reset::ResetPacket;
use self_test_result::SelfTestResultPacket;
use telemetry::TelemetryPacket;
use landed_telemetry::LandedTelemetryPacket;

pub mod ack;
pub mod amp_output_overwrite;
pub mod change_mode;
pub mod fire_pyro;
pub mod landed_telemetry;
pub mod low_power_telemetry;
pub mod reset;
pub mod self_test_result;
pub mod telemetry;
pub mod set_target_apogee;

// TODO change
pub const MAX_VLP_PACKET_SIZE: usize = 100;

// Shared by every packet that carries a temperature, so the same reading
// decodes identically whichever one it arrives on. Kept here rather than
// duplicated per packet because the two copies had drifted: the low-power
// packet's floor was 0 C, which silently clamped sub-freezing pad readings.
fixed_point_factory!(TemperatureFac, f32, -10.0, 85.0, 0.2);

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VLPDownlinkPacket {
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
            0 => AckPacket::deserialize(data).map(VLPDownlinkPacket::Ack),
            1 => {
                LowPowerTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LowPowerTelemetry)
            }
            2 => TelemetryPacket::deserialize(data).map(VLPDownlinkPacket::Telemetry),
            3 => SelfTestResultPacket::deserialize(data).map(VLPDownlinkPacket::SelfTestResult),
            4 => LandedTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LandedTelemetry),
            _ => None,
        }
    }

    pub fn packet_type(&self) -> u8 {
        match self {
            VLPDownlinkPacket::Ack(_) => 0,
            VLPDownlinkPacket::LowPowerTelemetry(_) => 1,
            VLPDownlinkPacket::Telemetry(_) => 2,
            VLPDownlinkPacket::SelfTestResult(_) => 3,
            VLPDownlinkPacket::LandedTelemetry(_) => 4,
        }
    }

    pub fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = self.packet_type();
        buffer = &mut buffer[1..];

        1 + match self {
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
    AMPOutputOverwrite(AMPOutputOverwritePacket),
    FirePyro(FirePyroPacket),
    SetTargetApogee(SetTargetApogeePacket)
}

impl VLPUplinkPacket {
    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => ChangeModePacket::deserialize(data).map(VLPUplinkPacket::ChangeMode),
            1 => ResetPacket::deserialize(data).map(VLPUplinkPacket::Reset),
            2 => {
                AMPOutputOverwritePacket::deserialize(data).map(VLPUplinkPacket::AMPOutputOverwrite)
            }
            3 => FirePyroPacket::deserialize(data).map(VLPUplinkPacket::FirePyro),
            4 => SetTargetApogeePacket::deserialize(data).map(VLPUplinkPacket::SetTargetApogee),
            _ => None,
        }
    }

    pub fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPUplinkPacket::ChangeMode(_) => 0,
            VLPUplinkPacket::Reset(_) => 1,
            VLPUplinkPacket::AMPOutputOverwrite(_) => 2,
            VLPUplinkPacket::FirePyro(_) => 3,
            VLPUplinkPacket::SetTargetApogee(_) => 4,
        };
        buffer = &mut buffer[1..];

        1 + match self {
            VLPUplinkPacket::ChangeMode(packet) => packet.serialize(buffer),
            VLPUplinkPacket::Reset(packet) => packet.serialize(buffer),
            VLPUplinkPacket::AMPOutputOverwrite(packet) => packet.serialize(buffer),
            VLPUplinkPacket::FirePyro(packet) => packet.serialize(buffer),
            VLPUplinkPacket::SetTargetApogee(packet) => packet.serialize(buffer),
        }
    }
}
