use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{amp_status::PowerOutputStatus, CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct PayloadEPSOutputStatus {
    #[packed_field(bits = "0..13")]
    pub current_ma: u16,
    #[packed_field(bits = "13..14")]
    pub overwrote: bool,
    #[packed_field(bits = "14..16", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "14")]
#[repr(C)]
pub struct PayloadEPSStatusMessage {
    pub battery1_mv: u16,
    /// Unit: 0.1C, e.g. 250 = 25C
    pub battery1_temperature_raw: u16,

    pub battery2_mv: u16,
    /// Unit: 0.1C, e.g. 250 = 25C
    pub battery2_temperature_raw: u16,

    #[packed_field(element_size_bytes = "2")]
    pub output_3v3: PayloadEPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_5v: PayloadEPSOutputStatus,
    #[packed_field(element_size_bytes = "2")]
    pub output_9v: PayloadEPSOutputStatus,
}

impl PayloadEPSStatusMessage {
    pub fn new(
        battery1_mv: u16,
        battery1_temperature: f32,
        battery2_mv: u16,
        battery2_temperature: f32,

        output_3v3: PayloadEPSOutputStatus,
        output_5v: PayloadEPSOutputStatus,
        output_9v: PayloadEPSOutputStatus,
    ) -> Self {
        Self {
            battery1_mv,
            battery1_temperature_raw: (battery1_temperature * 10.0) as u16,
            battery2_mv,
            battery2_temperature_raw: (battery2_temperature * 10.0) as u16,
            output_3v3,
            output_5v,
            output_9v,
        }
    }

    pub fn battery1_temperature(&self) -> f32 {
        self.battery1_temperature_raw as f32 / 10.0
    }

    pub fn battery2_temperature(&self) -> f32 {
        self.battery2_temperature_raw as f32 / 10.0
    }
}

impl CanBusMessage for PayloadEPSStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSStatus(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        let output_status_min = PayloadEPSOutputStatus {
            current_ma: 0,
            overwrote: false,
            status: PowerOutputStatus::Disabled,
        };
        let output_status_max = PayloadEPSOutputStatus {
            current_ma: 0x1FFF, // 13 bits
            overwrote: true,
            status: PowerOutputStatus::PowerBad,
        };
         let output_status_mid = PayloadEPSOutputStatus {
            current_ma: 100,
            overwrote: false,
            status: PowerOutputStatus::PowerGood,
        };

        vec![
            PayloadEPSStatusMessage {
                battery1_mv: 0,
                battery1_temperature_raw: 0,
                battery2_mv: 0,
                battery2_temperature_raw: 0,
                output_3v3: output_status_min.clone(),
                output_5v: output_status_min.clone(),
                output_9v: output_status_min.clone(),
            }
            .into(),
            PayloadEPSStatusMessage {
                battery1_mv: u16::MAX,
                battery1_temperature_raw: u16::MAX,
                battery2_mv: u16::MAX,
                battery2_temperature_raw: u16::MAX,
                output_3v3: output_status_max.clone(),
                output_5v: output_status_max.clone(),
                output_9v: output_status_max.clone(),
            }
            .into(),
             PayloadEPSStatusMessage {
                battery1_mv: 1000,
                battery1_temperature_raw: 1000,
                battery2_mv: 1000,
                battery2_temperature_raw: 1000,
                output_3v3: output_status_mid.clone(),
                output_5v: output_status_mid.clone(),
                output_9v: output_status_mid.clone(),
            }
            .into(),
        ]
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();
        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(create_test_messages(), "payload_eps_status");
    }
}
