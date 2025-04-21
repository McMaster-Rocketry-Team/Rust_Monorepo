use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct]
pub struct PayloadControlMessage {
    pub eps1_out_3v3_enable: bool,
    pub eps1_out_5v_enable: bool,
    pub eps1_out_9v_enable: bool,
    pub eps2_out_3v3_enable: bool,
    pub eps2_out_5v_enable: bool,
    pub eps2_out_9v_enable: bool,
    _padding: ReservedZero<packed_bits::Bits<2>>,
}

impl PayloadControlMessage {
    pub fn new(
        eps1_out_3v3_enable: bool,
        eps1_out_5v_enable: bool,
        eps1_out_9v_enable: bool,
        eps2_out_3v3_enable: bool,
        eps2_out_5v_enable: bool,
        eps2_out_9v_enable: bool,
    ) -> Self {
        Self {
            eps1_out_3v3_enable,
            eps1_out_5v_enable,
            eps1_out_9v_enable,
            eps2_out_3v3_enable,
            eps2_out_5v_enable,
            eps2_out_9v_enable,
            _padding: Default::default(),
        }
    }
}

impl CanBusMessage for PayloadControlMessage {
    fn len() -> usize {
        1
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
