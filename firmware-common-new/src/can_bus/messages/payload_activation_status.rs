use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::CanBusMessage;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct PayloadActivationStatusMessage {
  pub payload_esp_connected: bool,
  // TODO: more coming
}

impl PayloadActivationStatusMessage {
  pub fn new(payload_esp_connected: bool) -> Self {
    Self { payload_esp_connected }
  }
}

impl CanBusMessage for PayloadActivationStatusMessage {
  fn len() -> usize {
    1
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