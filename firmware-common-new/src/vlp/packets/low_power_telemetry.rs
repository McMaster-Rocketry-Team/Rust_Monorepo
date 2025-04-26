use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

fixed_point_factory!(BatteryVFac, f32, 5.0, 8.5, 0.001);
fixed_point_factory!(TemperatureFac, f32, -30.0, 85.0, 0.1);

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
pub struct LowPowerTelemetryPacket {
    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,
    gps_fixed: bool,

    amp_online: bool,

    #[packed_field(element_size_bits = "12")]
    battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,

    #[packed_field(element_size_bits = "11")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
}

impl LowPowerTelemetryPacket {
    pub fn new(
        num_of_fix_satellites: u8,
        gps_fixed: bool,
        amp_online: bool,
        battery_v: f32,
        air_temperature: f32,
    ) -> Self {
        Self {
            num_of_fix_satellites,
            gps_fixed,
            amp_online,
            battery_v: BatteryVFac::to_fixed_point_capped(battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
        }
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
    }

    pub fn gps_fixed(&self) -> bool {
        self.gps_fixed
    }

    pub fn amp_online(&self) -> bool {
        self.amp_online
    }

    pub fn battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.battery_v)
    }

    pub fn air_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.air_temperature)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for LowPowerTelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "LowPowerTelemetryPacket {{ num_of_fix_satellites: {}, gps_fixed: {}, amp_online: {}, battery_v: {}, air_temperature: {} }}",
            self.num_of_fix_satellites(),
            self.gps_fixed(),
            self.amp_online(),
            self.battery_v(),
            self.air_temperature(),
        )
    }
}