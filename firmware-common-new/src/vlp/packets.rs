use core::any::{Any, TypeId};
use core::fmt::Debug;

use ack::AckPacket;
use change_mode::ChangeModePacket;
use gps_beacon::GPSBeaconPacket;
use low_power_telemetry::LowPowerTelemetryPacket;
use packed_struct::prelude::*;
use packed_struct::types::bits::ByteArray;

pub mod ack;
pub mod change_mode;
pub mod gps_beacon;
pub mod low_power_telemetry;
pub mod self_test_result_packet;
pub mod telemetry_packet;

// TODO change
pub const MAX_VLP_PACKET_SIZE: usize = 100;

pub enum VLPDownlinkPacket {
    GPSBeacon(GPSBeaconPacket),
    Ack(AckPacket),
    LowPowerTelemetry(LowPowerTelemetryPacket),
}

impl VLPDownlinkPacket {
    pub(super) fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => {
                <GPSBeaconPacket as VLPPacket>::deserialize(data).map(VLPDownlinkPacket::GPSBeacon)
            }
            1 => <AckPacket as VLPPacket>::deserialize(data).map(VLPDownlinkPacket::Ack),
            2 => <LowPowerTelemetryPacket as VLPPacket>::deserialize(data)
                .map(VLPDownlinkPacket::LowPowerTelemetry),
            _ => None,
        }
    }
}

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
            0 => {
                <ChangeModePacket as VLPPacket>::deserialize(data).map(VLPUplinkPacket::ChangeMode)
            }
            _ => None,
        }
    }

    pub(super) fn serialize(self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPUplinkPacket::ChangeMode(_) => 0,
        };
        buffer = &mut buffer[1..];

        1+ match self {
            VLPUplinkPacket::ChangeMode(packet) => {
                packet.serialize(buffer);
                ChangeModePacket::len()
            }
        }
    }
}

pub trait VLPPacket: Clone + Debug + Any {
    fn len() -> usize;

    fn serialize(self, buffer: &mut [u8]);

    fn deserialize(data: &[u8]) -> Option<Self>;
}

impl<T> VLPPacket for T
where
    T: PackedStruct + Debug + Clone + Any,
    T::ByteArray: ByteArray,
{
    fn len() -> usize {
        T::packed_bytes_size(None).unwrap()
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..Self::len()]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
