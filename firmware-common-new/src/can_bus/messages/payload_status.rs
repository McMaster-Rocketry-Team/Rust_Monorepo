use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{amp_status::PowerOutputStatus, CanBusMessage};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct EPSOutputStatus {
    #[packed_field(bits = "0..14")]
    pub current_ma: u16,
    #[packed_field(bits = "14..16", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
#[repr(C)]
pub struct EPSStatus {
    pub battery1_mv: u16,
    pub battery2_mv: u16,

    #[packed_field(element_size_bytes = "2")]
    pub output_3v3: EPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_5v: EPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_9v: EPSOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "25")]
#[repr(C)]
pub struct PayloadStatusMessage {
    #[packed_field(element_size_bytes = "10")]
    pub eps1: EPSStatus,
    #[packed_field(element_size_bytes = "10")]
    pub eps2: EPSStatus,

    #[packed_field(element_size_bits = "12")]
    pub eps1_node_id: u16,
    #[packed_field(element_size_bits = "12")]
    pub eps2_node_id: u16,
    #[packed_field(element_size_bits = "12")]
    pub payload_esp_node_id: u16,

    #[packed_field(element_size_bits = "4")]
    _padding: u8,
}

impl PayloadStatusMessage {
    pub fn new(
        eps1: EPSStatus,
        eps2: EPSStatus,
        eps1_node_id: u16,
        eps2_node_id: u16,
        payload_esp_node_id: u16,
    ) -> Self {
        Self {
            eps1,
            eps2,
            eps1_node_id: eps1_node_id.into(),
            eps2_node_id: eps2_node_id.into(),
            payload_esp_node_id: payload_esp_node_id.into(),
            _padding: Default::default(),
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_payload_status_message() {
        let mut buffer = [0u8; 25];

        let message = PayloadStatusMessage::new(
            EPSStatus {
                battery1_mv: 1,
                battery2_mv: 2,
                output_3v3: EPSOutputStatus {
                    current_ma: 3,
                    status: PowerOutputStatus::Disabled,
                },
                output_5v: EPSOutputStatus {
                    current_ma: 4,
                    status: PowerOutputStatus::PowerGood,
                },
                output_9v: EPSOutputStatus {
                    current_ma: 5,
                    status: PowerOutputStatus::PowerBad,
                },
            },
            EPSStatus {
                battery1_mv: 6,
                battery2_mv: 7,
                output_3v3: EPSOutputStatus {
                    current_ma: 8,
                    status: PowerOutputStatus::Disabled,
                },
                output_5v: EPSOutputStatus {
                    current_ma: 9,
                    status: PowerOutputStatus::PowerGood,
                },
                output_9v: EPSOutputStatus {
                    current_ma: 10,
                    status: PowerOutputStatus::PowerBad,
                },
            },
            11,
            12,
            13,
        );

        message.serialize(&mut buffer);

        println!("{:?}", buffer);
    }
}
