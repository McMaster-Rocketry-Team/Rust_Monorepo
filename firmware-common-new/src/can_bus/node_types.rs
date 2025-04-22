// the lower the number, the higher the priority
// the maximum node type is 63

/// Main avionics node
pub const VOID_LAKE_NODE_TYPE: u8 = 5;

/// Node controlling the power distribution system
pub const AMP_NODE_TYPE: u8 = 10;

/// Air brakes node
pub const ICARUS_NODE_TYPE: u8 = 15;

/// Payload activation node
pub const PAYLOAD_ACTIVATION_NODE_TYPE: u8 = 20;

/// Rocket WiFi node in payload bay
pub const PAYLOAD_ROCKET_WIFI_NODE_TYPE: u8 = 21;

/// Strain gauges node
pub const OZYS_NODE_TYPE: u8 = 25;

/// Bulkhead node
pub const BULKHEAD_NODE_TYPE: u8 = 30;

/// EPS node in payload bay
pub const PAYLOAD_EPS_NODE_TYPE: u8 = 40;

/// Aero rust node
pub const AERO_RUST_NODE_TYPE: u8 = 50;

