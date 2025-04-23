use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputStatus {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct AmpStatusMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out1: PowerOutputStatus,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out2: PowerOutputStatus,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out3: PowerOutputStatus,
    #[packed_field(bits = "6..8", ty = "enum")]
    pub out4: PowerOutputStatus,
}

impl Into<u16> for AmpStatusMessage {
    fn into(self) -> u16 {
        let mut buffer = [0; 2];
        self.pack_to_slice(&mut buffer[..]).unwrap();
        u16::from_be_bytes(buffer)
    }
}

impl From<u16> for AmpStatusMessage {
    fn from(value: u16) -> Self {
        let buffer = value.to_be_bytes();
        Self::unpack_from_slice(&buffer[..]).unwrap()
    }
}