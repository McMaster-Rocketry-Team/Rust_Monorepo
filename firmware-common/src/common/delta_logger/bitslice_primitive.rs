use core::mem::transmute_copy;
use core::mem::transmute;
use core::mem::MaybeUninit;
use core::mem::ManuallyDrop;

use bitvec::prelude::*;
use num_traits::ToBytes;

use super::SerializeBitOrder;

pub trait BitSlicePrimitive {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>);

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self;

    fn len_bits() -> usize;
}

impl BitSlicePrimitive for bool {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        slice.set(0, *self);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        slice[0]
    }

    fn len_bits() -> usize {
        1
    }
}

impl BitSlicePrimitive for u8 {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        let data = [*self];
        let data: &BitSlice<u8, SerializeBitOrder> = data.view_bits();
        (&mut slice[..8]).copy_from_bitslice(data);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let slice = &slice[..8];
        slice.load_le::<u8>()
    }

    fn len_bits() -> usize {
        8
    }
}

impl BitSlicePrimitive for u32 {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        let data = self.to_le_bytes();
        let data: &BitSlice<u8, SerializeBitOrder> = data.view_bits();
        (&mut slice[..32]).copy_from_bitslice(data);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let slice = &slice[..32];
        slice.load_le::<u32>()
    }

    fn len_bits() -> usize {
        32
    }
}

impl BitSlicePrimitive for i64 {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        let data = self.to_le_bytes();
        let data: &BitSlice<u8, SerializeBitOrder> = data.view_bits();
        (&mut slice[..64]).copy_from_bitslice(data);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let slice = &slice[..64];
        slice.load_le::<i64>()
    }

    fn len_bits() -> usize {
        64
    }
}

impl BitSlicePrimitive for f32 {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        let data = self.to_le_bytes();
        let data: &BitSlice<u8, SerializeBitOrder> = data.view_bits();
        (&mut slice[..32]).copy_from_bitslice(data);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let slice = &slice[..32];
        unsafe { transmute(slice.load_le::<u32>()) }
    }

    fn len_bits() -> usize {
        32
    }
}

impl BitSlicePrimitive for f64 {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        let data = self.to_le_bytes();
        let data: &BitSlice<u8, SerializeBitOrder> = data.view_bits();
        (&mut slice[..64]).copy_from_bitslice(data);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let slice = &slice[..64];
        unsafe { transmute(slice.load_le::<u64>()) }
    }

    fn len_bits() -> usize {
        64
    }
}

impl<T: BitSlicePrimitive> BitSlicePrimitive for (T, T) {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        self.0.write(&mut slice[..T::len_bits()]);
        self.1.write(&mut slice[T::len_bits()..]);
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        (
            T::read(&slice[..T::len_bits()]),
            T::read(&slice[T::len_bits()..]),
        )
    }

    fn len_bits() -> usize {
        64 + 64
    }
}

impl<T: BitSlicePrimitive, const N: usize> BitSlicePrimitive for [T; N] {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        for i in 0..N {
            self[i].write(&mut slice[i * T::len_bits()..(i + 1) * T::len_bits()]);
        }
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        let mut result: [MaybeUninit<T>; N] = unsafe { MaybeUninit::uninit().assume_init() };
        for i in 0..N {
            result[i] = MaybeUninit::new(T::read(&slice[i * T::len_bits()..(i + 1) * T::len_bits()]));
        }
        unsafe { transmute_copy(&ManuallyDrop::new(result)) }
    }

    fn len_bits() -> usize {
        N * T::len_bits()
    }
}

impl<T: BitSlicePrimitive> BitSlicePrimitive for Option<T> {
    fn write(&self, slice: &mut BitSlice<u8, SerializeBitOrder>) {
        if let Some(value) = self {
            slice.set(0, true);
            value.write(&mut slice[1..]);
        } else {
            slice.set(0, false);
            (&mut slice[1..(T::len_bits() + 1)]).fill(true);
        }
    }

    fn read(slice: &BitSlice<u8, SerializeBitOrder>) -> Self {
        if slice[0] {
            Some(T::read(&slice[1..]))
        } else {
            None
        }
    }

    fn len_bits() -> usize {
        T::len_bits() + 1
    }
}
