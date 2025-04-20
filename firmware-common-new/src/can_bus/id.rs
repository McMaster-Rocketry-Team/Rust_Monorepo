use core::fmt::Debug;
use packed_struct::prelude::*;

#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "msb", size_bytes = "4")]
pub struct CanBusExtendedId {
    _reserved: ReservedZero<packed_bits::Bits<3>>,
    pub priority: Integer<u8, packed_bits::Bits<3>>,
    pub message_type: u8,
    pub node_type: Integer<u8, packed_bits::Bits<6>>,
    pub node_id: Integer<u16, packed_bits::Bits<12>>,
}

impl CanBusExtendedId {
    pub fn new(priority: u8, message_type: u8, node_type: u8, node_id: u16) -> Self {
        Self {
            _reserved: ReservedZero::default(),
            priority: priority.into(),
            message_type: message_type.into(),
            node_type: node_type.into(),
            node_id: node_id.into(),
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
