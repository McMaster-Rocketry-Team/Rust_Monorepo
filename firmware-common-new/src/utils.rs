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

pub const fn max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}

#[macro_export]
macro_rules! max_const {
    ($a:expr $(,)?) => { $a };
    ($a:expr, $b:expr $(,)?) => { $crate::utils::max($a, $b) };
    ($a:expr $(, $rest:expr)* $(,)?) => { $crate::utils::max($a, $crate::max_const!($($rest),+)) };
}
