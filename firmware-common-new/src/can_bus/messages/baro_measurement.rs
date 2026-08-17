use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "13")]
#[repr(C)]
pub struct BaroMeasurementMessage {
    pressure_raw: u32,

    /// Unit: 0.1C, e.g. 250 = 25C, -155 = -15.5C
    ///
    /// Signed. It was a `u16`, and Rust's float-to-int `as` saturates rather
    /// than wrapping, so every sub-zero reading landed on exactly 0 and
    /// downlinked as 0.0C — a plausible-looking number, not an obvious fault.
    /// The VLP layer already had to fix the same bug at its own end (see the
    /// `TemperatureFac` comment in `vlp/packets.rs`, whose floor was re-cut to
    /// -10C because a 0C floor "silently clamped sub-freezing pad readings");
    /// this is the CAN half of it, and without it the wider VLP range has
    /// nothing sub-zero to carry. `get_builtin_type_bit_width` maps `u16` and
    /// `i16` alike to 16, so the packed message is still 13 bytes.
    temperature_raw: i16,

    /// Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
    #[packed_field(element_size_bits = "56")]
    pub timestamp_us: u64,
}

impl BaroMeasurementMessage {
    pub fn new(timestamp_us: u64, pressure: f32, temperature: f32) -> Self {
        Self {
            pressure_raw: u32::from_be_bytes(pressure.to_be_bytes()),
            temperature_raw: (temperature * 10.0) as i16,
            timestamp_us,
        }
    }

    /// Pressure in Pa
    pub fn pressure(&self) -> f32 {
        f32::from_bits(self.pressure_raw)
    }

    /// Temperature in C
    pub fn temperature(&self) -> f32 {
        self.temperature_raw as f32 / 10.0
    }

    pub fn altitude_asl(&self) -> f32 {
        return calculate_isa_altitude(Pascals(self.pressure() as f64)).0 as f32;
    }
}

impl CanBusMessage for BaroMeasurementMessage {
    fn priority(&self) -> u8 {
        3
    }
}

impl Into<CanBusMessageEnum> for BaroMeasurementMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::BaroMeasurement(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            BaroMeasurementMessage::new(0, 0.0, 0.0).into(),
            BaroMeasurementMessage::new(
                0x00FFFFFFFFFFFFFF,
                f32::MAX,
                3276.7,
            )
            .into(),
            // A cold pad. The raw field used to be unsigned, which turned
            // every one of these into 0.0C.
            BaroMeasurementMessage::new(1, 101325.0, -15.5).into(),
        ]
    }

    /// The whole point of the signed raw field: sub-zero readings used to
    /// saturate to 0 on the way in and come back as a balmy 0.0C.
    #[test]
    fn sub_zero_temperatures_survive() {
        init_logger();

        assert_eq!(BaroMeasurementMessage::new(0, 0.0, -15.5).temperature(), -15.5);
        assert_eq!(BaroMeasurementMessage::new(0, 0.0, -40.0).temperature(), -40.0);
        // And a real zero is still a zero, not an underflowed negative.
        assert_eq!(BaroMeasurementMessage::new(0, 0.0, 0.0).temperature(), 0.0);
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();
        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(create_test_messages(), "baro_measurement");
    }

    #[test]
    fn altitude_calculation(){
        init_logger();

        log_info!("{}", BaroMeasurementMessage::new(0, 103325.3, 30.0).altitude_asl())
    }
}
