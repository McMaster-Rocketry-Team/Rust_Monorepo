use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

use super::VLPDownlinkPacket;

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);
fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);
fixed_point_factory!(TemperatureFac, f32, -30.0, 85.0, 0.1);
fixed_point_factory!(AltitudeFac, f32, -100.0, 5000.0, 1.0);

#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "12")]
pub struct GPSBeaconPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,

    #[packed_field(element_size_bits = "10")]
    battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,

    #[packed_field(element_size_bits = "11")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

    pub pyro_main_continuity: bool,
    pub pyro_main_fire: bool,
    pub pyro_drogue_continuity: bool,
    pub pyro_drogue_fire: bool,

    pub pyro_short_circuit: bool,
}

impl GPSBeaconPacket {
    pub fn new(
        nonce: u8,
        lat_lon: Option<(f64, f64)>,
        num_of_fix_satellites: u8,
        battery_v: f32,
        air_temperature: f32,
        altitude_agl: f32,
        pyro_main_continuity: bool,
        pyro_main_fire: bool,
        pyro_drogue_continuity: bool,
        pyro_drogue_fire: bool,
        pyro_short_circuit: bool,
    ) -> Self {
        Self {
            nonce: nonce.into(),
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),
            num_of_fix_satellites: num_of_fix_satellites,
            battery_v: BatteryVFac::to_fixed_point_capped(battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
            altitude_agl: AltitudeFac::to_fixed_point_capped(altitude_agl),
            pyro_main_continuity,
            pyro_main_fire,
            pyro_drogue_continuity,
            pyro_drogue_fire,
            pyro_short_circuit,
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

    pub fn air_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.air_temperature)
    }

    pub fn altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.altitude_agl)
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        let (lat, lon) = self.lat_lon();
        json::object! {
            lat: lat,
            lon: lon,
            num_of_fix_satellites: self.num_of_fix_satellites(),
            battery_v: self.battery_v(),
            air_temperature: self.air_temperature(),
            altitude_agl: self.altitude_agl(),
            pyro_main_continuity: self.pyro_main_continuity,
            pyro_main_fire: self.pyro_main_fire,
            pyro_drogue_continuity: self.pyro_drogue_continuity,
            pyro_drogue_fire: self.pyro_drogue_fire,
            pyro_short_circuit: self.pyro_short_circuit,
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for GPSBeaconPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "GPSBeaconPacket")
    }
}

impl Into<VLPDownlinkPacket> for GPSBeaconPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::GPSBeacon(self)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let packet = GPSBeaconPacket::new(
            10,
            Some((89.0, 179.0)),
            10,
            8.4,
            27.0,
            100.0,
            true,
            true,
            true,
            true,
            false,
        );
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();

        assert_eq!(deserialized_packet, packet);
    }
}
