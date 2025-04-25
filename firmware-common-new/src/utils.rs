use packed_struct::prelude::*;
use packed_struct::types::bits::ByteArray;
use core::fmt::Debug;

pub trait FixedLenSerializable: Clone + Debug {
    fn len() -> usize;

    fn serialize(self, buffer: &mut [u8]);

    fn deserialize(data: &[u8]) -> Option<Self>;
}

impl<T> FixedLenSerializable for T
where
    T: PackedStruct + Debug + Clone,
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
