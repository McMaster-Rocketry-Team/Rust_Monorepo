use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

use super::VLPDownlinkPacket;

fixed_point_factory!(BatteryVFac, f32, 5.0, 8.5, 0.02);
fixed_point_factory!(TemperatureFac, f32, -30.0, 85.0, 0.1);
fixed_point_factory!(AltitudeFac, f32, -100.0, 5000.0, 1.0);

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "4")]
pub struct AltimeterTelemetryPacket {
    #[packed_field(element_size_bits = "8")]
    vl_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
    #[packed_field(element_size_bits = "9")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    altitude: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
}

impl AltimeterTelemetryPacket {
    pub fn new(vl_battery_v: f32, air_temperature: f32, altitude: f32) -> Self {
        Self {
            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
            altitude: AltitudeFac::to_fixed_point_capped(altitude),
        }
    }

    pub fn vl_battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.vl_battery_v)
    }

    pub fn air_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.air_temperature)
    }

    pub fn altitude(&self) -> f32 {
        AltitudeFac::to_float(self.altitude)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AltimeterTelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "AltimeterTelemetryPacket {{ vl_battery_v: {}, air_temperature: {}, altitude: {} }}",
            self.vl_battery_v(),
            self.air_temperature(),
            self.altitude()
        )
    }
}

impl Into<VLPDownlinkPacket> for AltimeterTelemetryPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::AltimeterTelemetry(self)
    }
}
