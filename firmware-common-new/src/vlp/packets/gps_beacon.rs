use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);
fixed_point_factory!(BatteryVFac, f32, 5.0, 8.5, 0.001);

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "8")]
pub struct GPSBeaconPacket {
    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,

    #[packed_field(element_size_bits = "12")]
    battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
}

impl GPSBeaconPacket {
    pub fn new(lat_lon: Option<(f64, f64)>, num_of_fix_satellites: u8, battery_v: f32) -> Self {
        Self {
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),
            num_of_fix_satellites: num_of_fix_satellites,
            battery_v: BatteryVFac::to_fixed_point_capped(battery_v),
        }
    }

    pub fn lat_lon(&self) -> (f64, f64) {
        (LatFac::to_float(self.lat), LonFac::to_float(self.lon))
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
    }

    pub fn battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.battery_v)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for GPSBeaconPacket {
    fn format(&self, f: defmt::Formatter) {
        let (lat, lon) = self.lat_lon();
        defmt::write!(
           f,
           "GPSBeaconPacket {{ lat: {}, lon: {}, num_of_fix_satellites: {}, battery_v: {} }}",
           lat,
           lon,
           self.num_of_fix_satellites(),
           self.battery_v(),
        )
    }
}