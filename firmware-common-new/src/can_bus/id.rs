use core::fmt::Debug;
use packed_struct::prelude::*;

use super::messages::{CanBusMessage, CanBusMessageEnum};

#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct CanBusExtendedId {
    #[packed_field(element_size_bits = "3")]
    _reserved: u8,

    #[packed_field(element_size_bits = "3")]
    pub priority: u8,

    pub message_type: u8,
    #[packed_field(element_size_bits = "6")]
    pub node_type: u8,

    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl CanBusExtendedId {
    pub fn new(priority: u8, message_type: u8, node_type: u8, node_id: u16) -> Self {
        Self {
            _reserved: Default::default(),
            priority: priority,
            message_type: message_type,
            node_type: node_type,
            node_id: node_id,
        }
    }

    pub fn from_message<T: CanBusMessage>(message: &T, node_type: u8, node_id: u16) -> Self {
        Self::new(
            message.priority(),
            CanBusMessageEnum::get_message_type::<T>().unwrap(),
            node_type,
            node_id,
        )
    }

    pub fn from_raw(raw: u32) -> Self {
        let unpacked = raw.to_be_bytes();
        let mut packed = [0; 4];
        packed.copy_from_slice(&unpacked);
        Self::unpack(&packed).unwrap()
    }
}

impl Into<u32> for CanBusExtendedId {
    fn into(self) -> u32 {
        let packed = self.pack().unwrap();
        u32::from_be_bytes(packed)
    }
}
