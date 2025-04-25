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
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "34")]
#[repr(C)]
pub struct DataTransferMessage {
    data: [u8; 32],
    data_len: u8,
    pub start_of_data: bool,
    pub end_of_data: bool,
    #[packed_field(bits = "266..268", ty = "enum")]
    pub data_type: DataType,
    #[packed_field(element_size_bits = "4")]
    _padding: u8,
}

impl DataTransferMessage {
    pub fn new(mut data: Vec<u8, 32>, data_type: DataType, start_of_data:bool, end_of_data: bool) -> Self {
        let data_len = data.len() as u8;
        data.resize_default(32).unwrap();
        Self {
            data_len,
            data: data.into_array().unwrap(),
            data_type,
            start_of_data,
            end_of_data,
            _padding: Default::default(),
        }
    }

    pub fn data(&self) -> &[u8] {
        let data_len: u8 = self.data_len.into();
        &self.data[..(data_len as usize)]
    }
}

impl CanBusMessage for DataTransferMessage {
    fn priority(&self) -> u8 {
        7
    }
}
