use core::fmt::Debug;
use packed_struct::prelude::*;
use packed_struct::types::bits::ByteArray;

pub trait FixedLenSerializable: Clone + Debug {
    fn serialized_len() -> usize;

    fn serialize(&self, buffer: &mut [u8]) -> usize;

    fn deserialize(data: &[u8]) -> Option<Self>;
}

impl<T> FixedLenSerializable for T
where
    T: PackedStruct + Debug + Clone,
    T::ByteArray: ByteArray,
{
    fn serialized_len() -> usize {
        T::packed_bytes_size(None).unwrap()
    }

    fn serialize(&self, buffer: &mut [u8]) -> usize {
        let len = T::packed_bytes_size(None).unwrap();
        self.pack_to_slice(&mut buffer[..len]).unwrap();
        len
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
