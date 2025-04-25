use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct]
#[repr(C)]
pub struct PayloadControlMessage {
    pub eps1_out_3v3_enable: bool,
    pub eps1_out_5v_enable: bool,
    pub eps1_out_9v_enable: bool,
    pub eps2_out_3v3_enable: bool,
    pub eps2_out_5v_enable: bool,
    pub eps2_out_9v_enable: bool,
    #[packed_field(element_size_bits = "2")]
    _padding: u8,
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
    fn priority(&self) -> u8 {
        5
    }
}
