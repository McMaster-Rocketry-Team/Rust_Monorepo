use crate::utils::FixedLenSerializable;
use core::fmt::Debug;
use packed_struct::prelude::*;
use packed_struct::types::bits::ByteArray;

pub mod ozys_custom_status;
pub mod vl_custom_status;

pub trait NodeCustomStatus {}

pub trait NodeCustomStatusExt {
    fn to_u16(&self) -> u16;

    fn from_u16(status: u16) -> Self;
}

impl<T> NodeCustomStatusExt for T
where
    T: NodeCustomStatus,
    T: PackedStruct + Debug + Clone,
    T::ByteArray: ByteArray,
{
    fn to_u16(&self) -> u16 {
        let mut buffer = [0u8; 2];
        self.serialize(&mut buffer);
        u16::from_be_bytes(buffer) >> 5
    }

    fn from_u16(status: u16) -> Self {
        Self::deserialize(&(status << 5).to_be_bytes()).unwrap()
    }
}
