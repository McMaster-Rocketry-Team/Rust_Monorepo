use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct]
#[repr(C)]
pub struct PayloadEPSControlMessage {
    pub out_3v3_enable: bool,
    pub out_5v_enable: bool,
    pub out_9v_enable: bool,
    #[packed_field(element_size_bits = "5")]
    _padding: u8,
}

impl PayloadEPSControlMessage {
    pub fn new(
        out_3v3_enable: bool,
        out_5v_enable: bool,
        out_9v_enable: bool,
    ) -> Self {
        Self {
            out_3v3_enable,
            out_5v_enable,
            out_9v_enable,
            _padding: Default::default(),
        }
    }
}

impl CanBusMessage for PayloadEPSControlMessage {
    fn priority(&self) -> u8 {
        5
    }
}
