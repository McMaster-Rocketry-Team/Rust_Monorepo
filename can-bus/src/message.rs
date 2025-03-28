use core::fmt::Debug;
use heapless::Vec;
use packed_bits::ByteArray;
use packed_struct::prelude::*;

use super::id::CanBusExtendedId;

pub trait CanBusMessage: PackedStruct + Clone + Debug {
    fn message_type() -> u8;

    fn create_id(priority: u8, node_type: u8, node_id: u16) -> CanBusExtendedId {
        CanBusExtendedId::new(priority, Self::message_type(), node_type, node_id)
    }

    fn to_data(&self) -> Vec<u8, 8> {
        let packed = self.pack().unwrap();
        Vec::from_slice(packed.as_bytes_slice()).unwrap()
    }

    fn from_data(data: &Self::ByteArray) -> Self {
        Self::unpack(data).unwrap()
    }

    fn len() -> usize {
        Self::ByteArray::len()
    }
}
