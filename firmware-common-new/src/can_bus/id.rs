use core::fmt::Debug;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::sender::CAN_CRC;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
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

pub fn can_node_id_from_serial_number(serial_number: &[u8]) -> u16 {
    CAN_CRC.checksum(serial_number) & 0xFFF
}
