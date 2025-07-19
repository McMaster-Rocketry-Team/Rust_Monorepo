use packed_struct::prelude::*;

#[derive(PackedStruct, Clone, Debug)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "13")]
pub struct UsbCanBusFrame {
    pub id: u32,
    pub data_length: u8,
    pub data: [u8; 8],
}

impl UsbCanBusFrame {
    pub const SERIALIZED_SIZE: usize = 13;

    pub fn new(id: u32, input_data: &[u8]) -> Self {
        let data_length = input_data.len().min(8);
        let mut data = [0u8; 8];
        data[..data_length].copy_from_slice(&input_data[..data_length]);
        Self {
            id,
            data_length: data_length as u8,
            data,
        }
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..(self.data_length as usize)]
    }
}
