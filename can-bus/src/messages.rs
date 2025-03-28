use packed_struct::prelude::*;

use super::message::CanBusMessage;

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
pub struct UnixTimeMessage {
    /// Current milliseconds since Unix epoch, floored to the nearest ms
    pub timestamp: Integer<u64, packed_bits::Bits<48>>,
}

impl CanBusMessage for UnixTimeMessage {
    fn message_type() -> u8 {
        0
    }
}

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct AvionicsStatusMessage {
    #[packed_field(bits = "0")]
    pub low_power: bool,
    #[packed_field(bits = "1")]
    pub armed: bool,
}

impl CanBusMessage for AvionicsStatusMessage {
    fn message_type() -> u8 {
        1
    }
}

#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightEvent {
    Ignition = 0,
    Coast = 1,
    Apogee = 2,
    Landed = 3,
}

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "7")]
pub struct FlightEventMessage {
    /// Current milliseconds since Unix epoch, floored to the nearest ms
    #[packed_field(bits = "0..48")]
    pub timestamp: Integer<u64, packed_bits::Bits<48>>,

    #[packed_field(bits = "48..=50", ty = "enum")]
    pub event: FlightEvent,
}

impl CanBusMessage for FlightEventMessage {
    fn message_type() -> u8 {
        2
    }
}

#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq)]
pub enum HealthState {
    Healthy = 0,
    Degraded = 1,
    UnHealthy = 2,
}

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct HealthMessage {
    #[packed_field(bits = "0..=1", ty = "enum")]
    pub state: HealthState,
}

impl CanBusMessage for HealthMessage {
    fn message_type() -> u8 {
        3
    }
}

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct ResetMessage {
}

impl CanBusMessage for ResetMessage {
    fn message_type() -> u8 {
        4
    }
}