use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "10")]
pub struct BaroMeasurementMessage {
  pressure: u32,

  /// Measurement timestamp, milliseconds since Unix epoch, floored to the nearest ms
  pub timestamp: Integer<u64, packed_bits::Bits<48>>,
}


impl BaroMeasurementMessage {
  pub fn new(
    timestamp: f64,
    pressure: f32,
  ) -> Self {
    Self {
      pressure: u32::from_be_bytes(pressure.to_be_bytes()),
      timestamp: (timestamp as u64).into(),
    }
  }

  /// Pressure in Pa
  pub fn pressure(&self) -> f32 {
    f32::from_bits(self.pressure)
  }

  pub fn altitude(&self) -> f32 {
    return calculate_isa_altitude(Pascals(self.pressure as f64)).0 as f32;
}
}

impl CanBusMessage for BaroMeasurementMessage {
  fn len() -> usize {
    10
  }

  fn priority(&self) -> u8 {
    6
  }

  fn serialize(self, buffer: &mut [u8]) {
    self.pack_to_slice(buffer).unwrap();
  }

  fn deserialize(data: &[u8]) -> Option<Self> {
    Self::unpack_from_slice(data).ok()
  }
}