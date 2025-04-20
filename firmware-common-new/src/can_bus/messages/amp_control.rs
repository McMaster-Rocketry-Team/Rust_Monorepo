use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct]
pub struct AmpControlMessage {
    out1_enable: bool,
    out2_enable: bool,
    out3_enable: bool,
    out4_enable: bool,
    _padding: ReservedZero<packed_bits::Bits<4>>,
}

impl AmpControlMessage {
    pub fn new(out1_enable: bool, out2_enable: bool, out3_enable: bool, out4_enable: bool) -> Self {
        Self {
            out1_enable,
            out2_enable,
            out3_enable,
            out4_enable,
            _padding: Default::default(),
        }
    }
}

impl CanBusMessage for AmpControlMessage {
    fn len() -> usize {
        1
    }

    fn priority() -> u8 {
        5
    }

    fn serialize(self, buffer: &mut [u8]) {
        self.pack_to_slice(buffer).unwrap();
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Self::unpack_from_slice(data).ok()
    }
}
