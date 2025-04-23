use core::any::{Any, TypeId};
use core::fmt::Debug;

use gps_beacon::GPSBeaconPacket;
use packed_struct::prelude::*;
use packed_struct::types::bits::ByteArray;

pub mod gps_beacon;
pub mod self_test_result_packet;
pub mod telemetry_packet;
pub mod change_mode;

pub enum VLPPacketEnum {
    GPSBeacon(GPSBeaconPacket),
}

impl VLPPacketEnum {
    pub(super) fn get_packet_type<T: VLPPacket>() -> Option<u8> {
        let t_id = TypeId::of::<T>();
        if t_id == TypeId::of::<GPSBeaconPacket>() {
            Some(0)
        } else {
            None
        }
    }

    pub(super) fn deserialize(packet_type: u8, data: &[u8]) -> Option<Self> {
        match packet_type {
            0 => <GPSBeaconPacket as VLPPacket>::deserialize(data).map(VLPPacketEnum::GPSBeacon),
            _ => None,
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
