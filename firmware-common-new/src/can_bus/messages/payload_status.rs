use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EPSOutputStatusEnum {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb")]
pub struct EPSOutputStatus {
    #[packed_field(bits = "0..14")]
    pub current_ma: Integer<u16, packed_bits::Bits<14>>,
    #[packed_field(bits = "14..16", ty = "enum")]
    pub status: EPSOutputStatusEnum,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
pub struct EPSStatus {
    #[packed_field(bits = "0..16")]
    pub battery1_mv: u16,
    pub battery2_mv: u16,

    #[packed_field(element_size_bits = "16")]
    pub output_3v3: EPSOutputStatus,
    #[packed_field(element_size_bits = "16")]
    pub output_5v: EPSOutputStatus,
    #[packed_field(element_size_bits = "16")]
    pub output_9v: EPSOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "25")]
pub struct PayloadStatusMessage {
    #[packed_field(element_size_bytes = "10")]
    pub eps1: EPSStatus,
    #[packed_field(element_size_bytes = "10")]
    pub eps2: EPSStatus,
    pub eps1_node_id: Integer<u16, packed_bits::Bits<12>>,
    pub eps2_node_id: Integer<u16, packed_bits::Bits<12>>,
    pub payload_esp_node_id: Integer<u16, packed_bits::Bits<12>>,
    _padding: ReservedZero<packed_bits::Bits<4>>,
}

impl PayloadStatusMessage {
    pub fn new(payload_esp_connected: bool) -> Self {
        todo!()
    }
}

impl CanBusMessage for PayloadStatusMessage {
    fn len() -> usize {
        25
    }

    fn priority(&self) -> u8 {
        5
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
