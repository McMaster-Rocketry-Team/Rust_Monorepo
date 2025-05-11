#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;

use super::{
    messages::{RESET_MESSAGE_TYPE, UNIX_TIME_MESSAGE_TYPE},
    sender::CAN_CRC,
};
use core::fmt::Debug;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub(crate) struct CanBusMessageTypeFlag {
    pub is_measurement: bool,
    pub is_control: bool,
    pub is_status: bool,
    pub is_data: bool,
    pub is_misc: bool,
}

pub(crate) const fn create_can_bus_message_type(flag: CanBusMessageTypeFlag, sub_type: u8) -> u8 {
    let mut message_type = 0;

    if flag.is_measurement {
        message_type |= 0b10000000;
    }
    if flag.is_control {
        message_type |= 0b01000000;
    }
    if flag.is_status {
        message_type |= 0b00100000;
    }
    if flag.is_data {
        message_type |= 0b00010000;
    }
    if flag.is_misc {
        message_type |= 0b00001000;
    }

    message_type |= sub_type & 0b00000111;

    message_type
}

/// Returns a mask
///
/// Filter logic: `frame_accepted = (incoming_id & mask) == 0`
///
/// - If the message type of the incoming frame is in `accept_message_types`, the frame will be accepted
/// - If the message type of the incoming frame is not in `accept_message_types`, the frame *MAY OR MAY NOT* be rejected
/// - `ResetMessage` and `UnixTimeMessage` is always accepted even if its not in the `accept_message_types` list
///
/// This is useful when you want to utilize the filter function of the CAN hardware.
pub fn create_can_bus_message_type_filter_mask(accept_message_types: &[u8]) -> u32 {
    let mut accept_message_type_ored = 0u8;
    for message_type in accept_message_types {
        accept_message_type_ored |= message_type;
    }
    accept_message_type_ored |= RESET_MESSAGE_TYPE;
    accept_message_type_ored |= UNIX_TIME_MESSAGE_TYPE;

    CanBusExtendedId::new(0, !accept_message_type_ored, 0, 0).into()
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", wasm_bindgen)]
#[derive(PackedStruct, Default, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[packed_struct(endian = "msb", size_bytes = "4")]
#[repr(C)]
pub struct CanBusExtendedId {
    #[packed_field(element_size_bits = "3")]
    _reserved: u8,

    #[packed_field(element_size_bits = "3")]
    pub priority: u8,

    pub message_type: u8,

    #[packed_field(element_size_bits = "6")]
    pub node_type: u8,

    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

#[cfg_attr(feature = "wasm", wasm_bindgen)]
impl CanBusExtendedId {
    #[cfg_attr(feature = "wasm", wasm_bindgen(constructor))]
    pub fn new(priority: u8, message_type: u8, node_type: u8, node_id: u16) -> Self {
        Self {
            _reserved: Default::default(),
            priority: priority,
            message_type: message_type,
            node_type: node_type,
            node_id: node_id,
        }
    }

    pub fn from_raw(raw: u32) -> Self {
        let unpacked = raw.to_be_bytes();
        let mut packed = [0; 4];
        packed.copy_from_slice(&unpacked);
        Self::unpack(&packed).unwrap()
    }
}

impl Into<u32> for CanBusExtendedId {
    fn into(self) -> u32 {
        let packed = self.pack().unwrap();
        u32::from_be_bytes(packed)
    }
}

pub fn can_node_id_from_serial_number(serial_number: &[u8]) -> u16 {
    CAN_CRC.checksum(serial_number) & 0xFFF
}

#[cfg(test)]
mod tests {
    use crate::{
        can_bus::messages::{
            ACK_MESSAGE_TYPE, AMP_STATUS_MESSAGE_TYPE, BARO_MEASUREMENT_MESSAGE_TYPE,
            DATA_TRANSFER_MESSAGE_TYPE, RESET_MESSAGE_TYPE, UNIX_TIME_MESSAGE_TYPE,
        },
        tests::init_logger,
    };

    use super::*;

    #[test]
    fn test_create_can_bus_message_type_filter_mask() {
        init_logger();

        let mask = create_can_bus_message_type_filter_mask(&[
            BARO_MEASUREMENT_MESSAGE_TYPE,
            DATA_TRANSFER_MESSAGE_TYPE,
        ]);

        let incoming_id = CanBusExtendedId::new(5, BARO_MEASUREMENT_MESSAGE_TYPE, 10, 20);
        let incoming_id: u32 = incoming_id.into();
        assert_eq!(incoming_id & mask, 0);

        let incoming_id = CanBusExtendedId::new(1, DATA_TRANSFER_MESSAGE_TYPE, 20, 30);
        let incoming_id: u32 = incoming_id.into();
        assert_eq!(incoming_id & mask, 0);

        let incoming_id = CanBusExtendedId::new(1, RESET_MESSAGE_TYPE, 20, 30);
        let incoming_id: u32 = incoming_id.into();
        assert_eq!(incoming_id & mask, 0);

        let incoming_id = CanBusExtendedId::new(1, UNIX_TIME_MESSAGE_TYPE, 20, 30);
        let incoming_id: u32 = incoming_id.into();
        assert_eq!(incoming_id & mask, 0);

        let incoming_id = CanBusExtendedId::new(1, ACK_MESSAGE_TYPE, 20, 30);
        let incoming_id: u32 = incoming_id.into();
        assert_ne!(incoming_id & mask, 0);

        let incoming_id = CanBusExtendedId::new(1, AMP_STATUS_MESSAGE_TYPE, 20, 30);
        let incoming_id: u32 = incoming_id.into();
        assert_ne!(incoming_id & mask, 0);
    }
}
