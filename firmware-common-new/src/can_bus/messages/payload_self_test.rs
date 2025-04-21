use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb")]
pub struct EPSSelfTestResult {
    #[packed_field(bits = "0")]
    pub battery1_ok: bool,
    pub battery2_ok: bool,
    pub out_3v3_ok: bool,
    pub out_5v_ok: bool,
    pub out_9v_ok: bool,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
pub struct PayloadSelfTestMessage {
    #[packed_field(element_size_bits = "5")]
    pub eps1: EPSSelfTestResult,
    #[packed_field(element_size_bits = "5")]
    pub eps2: EPSSelfTestResult,
    _padding: ReservedZero<packed_bits::Bits<6>>,
}

impl PayloadSelfTestMessage {
    pub fn new() -> Self {
        todo!()
    }
}

impl CanBusMessage for PayloadSelfTestMessage {
    fn len() -> usize {
        2
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
