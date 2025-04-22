use super::CanBusMessage;
use heapless::Vec;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum DataType {
    Firmware = 0,
    Data = 1,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "38")]
#[repr(C)]
pub struct DataTransferMessage {
    data: [u8; 32],
    data_len: u8,
    pub data_start_i: u32,
    pub end_of_data: bool,
    #[packed_field(bits = "297..299", ty = "enum")]
    pub data_type: DataType,
    #[packed_field(element_size_bits = "5")]
    _reserved: u8,
}

impl DataTransferMessage {
    pub fn new(data_start_i: u32, mut data: Vec<u8, 32>, data_type: DataType, end_of_data: bool) -> Self {
        for _ in data.len()..32 {
            data.push(0).unwrap();
        }
        Self {
            data_len: data.len() as u8,
            data: data.into_array().unwrap(),
            data_type,
            data_start_i,
            end_of_data,
            _reserved: Default::default(),
        }
    }

    pub fn data(&self) -> &[u8] {
        let data_len: u8 = self.data_len.into();
        &self.data[..(data_len as usize)]
    }
}

impl CanBusMessage for DataTransferMessage {
    fn len() -> usize {
        38
    }

    fn priority(&self) -> u8 {
        7
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(&mut buffer[..Self::len()]).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
