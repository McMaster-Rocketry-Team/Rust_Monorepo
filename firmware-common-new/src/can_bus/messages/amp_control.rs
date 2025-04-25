use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct]
#[repr(C)]
pub struct AmpControlMessage {
    pub out1_enable: bool,
    pub out2_enable: bool,
    pub out3_enable: bool,
    pub out4_enable: bool,

    #[packed_field(element_size_bits = "4")]
    _padding: u8,
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
    fn priority(&self) -> u8 {
        5
    }
}
