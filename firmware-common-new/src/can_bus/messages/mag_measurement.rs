use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "19")]
#[repr(C)]
pub struct MagMeasurementMessage {
    mag_raw: [u32; 3],

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl MagMeasurementMessage {
    pub fn new(timestamp_us: u64, mag: &[f32; 3]) -> Self {
        Self {
            mag_raw: mag.map(|x| u32::from_be_bytes(x.to_be_bytes())),
            timestamp_us,
        }
    }

    /// unit: tesla
    pub fn mag(&self) -> [f32; 3] {
        self.mag_raw.map(f32::from_bits)
    }
}

impl CanBusMessage for MagMeasurementMessage {
    fn priority(&self) -> u8 {
        3
    }
}

impl Into<CanBusMessageEnum> for MagMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::MagMeasurement(self)
    }
}

#[cfg(test)]
mod test {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};
    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            MagMeasurementMessage::new(0, &[0.0; 3]).into(),
            MagMeasurementMessage::new(
                0x00FFFFFFFFFFFFFF,
                &[f32::MAX; 3],
            )
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
        can_bus_messages_test::create_reference_data(create_test_messages(), "mag_measurement");
    }
}
