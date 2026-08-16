use core::cell::{RefCell, RefMut};

use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

use super::{TEMPERATURE_FAC_BITS, TemperatureFac, TemperatureFacBase, VLPDownlinkPacket};

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator (same facs as `TelemetryPacket`)
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);

fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);

// 87 bits = 11 bytes, 1 spare bit.
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "11")]
pub struct LowPowerTelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,
    pub gps_fixed: bool,

    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "10")]
    vl_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,

    pub amp_online: bool,
    #[packed_field(element_size_bits = "10")]
    shared_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,

    #[packed_field(element_size_bits = "9")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
}

impl LowPowerTelemetryPacket {
    pub fn new(
        nonce: u8,
        num_of_fix_satellites: u8,
        gps_fixed: bool,
        lat_lon: Option<(f64, f64)>,
        vl_battery_v: f32,
        amp_online: bool,
        shared_battery_v: f32,
        air_temperature: f32,
    ) -> Self {
        Self {
            nonce: nonce.into(),
            num_of_fix_satellites,
            gps_fixed,
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),
            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            amp_online,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
        }
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
    }

    pub fn lat(&self) -> f64 {
        LatFac::to_float(self.lat)
    }

    pub fn lon(&self) -> f64 {
        LonFac::to_float(self.lon)
    }

    pub fn vl_battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.vl_battery_v)
    }

    pub fn shared_battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.shared_battery_v)
    }

    pub fn air_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.air_temperature)
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            num_of_fix_satellites: self.num_of_fix_satellites(),
            gps_fixed: self.gps_fixed,
            lat: self.lat(),
            lon: self.lon(),
            vl_battery_v: self.vl_battery_v(),
            amp_online: self.amp_online,
            shared_battery_v: self.shared_battery_v(),
            air_temperature: self.air_temperature(),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for LowPowerTelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "LowPowerTelemetryPacket")
    }
}

impl Into<VLPDownlinkPacket> for LowPowerTelemetryPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::LowPowerTelemetry(self)
    }
}

pub struct LowPowerTelemetryPacketBuilderState {
    nonce: u8,
    pub num_of_fix_satellites: u8,
    pub gps_fixed: bool,
    pub lat_lon: Option<(f64, f64)>,
    pub vl_battery_v: f32,
    pub amp_online: bool,
    pub shared_battery_v: f32,
    pub air_temperature: f32,
}

pub struct LowPowerTelemetryPacketBuilder<M: RawMutex> {
    state: BlockingMutex<M, RefCell<LowPowerTelemetryPacketBuilderState>>,
}

impl<M: RawMutex> LowPowerTelemetryPacketBuilder<M> {
    pub fn new() -> Self {
        Self {
            state: BlockingMutex::new(RefCell::new(LowPowerTelemetryPacketBuilderState {
                nonce: 0,
                num_of_fix_satellites: 0,
                gps_fixed: false,
                lat_lon: None,
                vl_battery_v: 0.0,
                amp_online: false,
                shared_battery_v: 0.0,
                air_temperature: 0.0,
            })),
        }
    }

    pub fn create_packet(&self) -> LowPowerTelemetryPacket {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.nonce += 1;
            if state.nonce > 15 {
                state.nonce = 0;
            }
            LowPowerTelemetryPacket::new(
                state.nonce,
                state.num_of_fix_satellites,
                state.gps_fixed,
                state.lat_lon,
                state.vl_battery_v,
                state.amp_online,
                state.shared_battery_v,
                state.air_temperature,
            )
        })
    }

    pub fn update<U>(&self, update_fn: U)
    where
        U: FnOnce(&mut RefMut<LowPowerTelemetryPacketBuilderState>) -> (),
    {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            update_fn(&mut state);
        })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let packet = LowPowerTelemetryPacket::new(12, 5, true, Some((45.5, -73.6)), 8.1, true, 8.2, 27.0);
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);
        // 1 byte packet type + the 11 byte packed struct.
        assert_eq!(len, 12);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();
        assert_eq!(deserialized_packet, packet);

        let VLPDownlinkPacket::LowPowerTelemetry(p) = deserialized_packet else {
            unreachable!()
        };
        assert_relative_eq!(p.lat(), 45.5, epsilon = 0.0001);
        assert_relative_eq!(p.lon(), -73.6, epsilon = 0.0001);
    }
}
