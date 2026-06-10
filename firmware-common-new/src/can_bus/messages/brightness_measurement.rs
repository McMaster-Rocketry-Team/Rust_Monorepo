use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "11")]
#[repr(C)]
pub struct BrightnessMeasurementMessage {
    brightness_lux_raw: u32,

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl BrightnessMeasurementMessage {
    pub fn new(timestamp_us: u64, brightness_lux: f32) -> Self {
        Self {
            brightness_lux_raw: u32::from_be_bytes(brightness_lux.to_be_bytes()),
            timestamp_us,
        }
    }

    /// Brightness in Lux
    pub fn brightness_lux(&self) -> f32 {
        f32::from_bits(self.brightness_lux_raw)
    }
}

impl CanBusMessage for BrightnessMeasurementMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for BrightnessMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::BrightnessMeasurement(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            BrightnessMeasurementMessage::new(0, 0.0).into(),
            BrightnessMeasurementMessage::new(0x00FFFFFFFFFFFFFF, f32::MAX).into(),
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "brightness_measurement");
    }
}
