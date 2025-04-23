use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

fixed_point_factory!(BatteryVFac, f32, 5.0, 8.5, 0.001);

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "19")]
pub struct GPSBeaconPacket {
    lat: u64,
    lon: u64,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,

    #[packed_field(element_size_bits = "12")]
    battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
}

impl GPSBeaconPacket {
    pub fn new(lat: f64, lon: f64, num_of_fix_satellites: u8, battery_v: f32) -> Self {
        Self {
            lat: u64::from_be_bytes(lat.to_be_bytes()),
            lon: u64::from_be_bytes(lon.to_be_bytes()),
            num_of_fix_satellites: num_of_fix_satellites,
            battery_v: BatteryVFac::to_fixed_point_capped(battery_v),
        }
    }

    pub fn lat(&self) -> f64 {
        f64::from_bits(self.lat)
    }

    pub fn lon(&self) -> f64 {
        f64::from_bits(self.lon)
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
    }

    pub fn battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.battery_v)
    }
}
