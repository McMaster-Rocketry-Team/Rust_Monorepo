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
    /// Whether `lat` / `lon` hold a position. With no fix those two fields
    /// still carry the 0.0 filler, so this bit is the only thing separating a
    /// real equatorial position from Null Island.
    gps_fixed: bool,

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
            // `gps_fixed` is the validity bit for the position, so it cannot be
            // allowed to claim a fix the packet has no coordinates for. Every
            // caller already passes `gps_data.lat_lon.is_some()`; the `&&`
            // makes that agreement structural instead of a convention two
            // firmware call sites happen to follow.
            gps_fixed: gps_fixed && lat_lon.is_some(),
            // The 0.0 filler is never read back: `lat_lon` refuses to decode
            // unless `gps_fixed` is set.
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

    /// Whether the GPS had solved a position when this packet was built. For a
    /// coordinate use [`LowPowerTelemetryPacket::lat_lon`], which applies this
    /// bit for you; this getter is for displaying fix state on its own.
    pub fn gps_fixed(&self) -> bool {
        self.gps_fixed
    }

    /// `None` until the GPS has solved a position. One getter rather than a
    /// `lat()` and a `lon()`, for the same reason as on the other two
    /// telemetry packets: a coordinate is only useful whole, and there should
    /// be exactly one place the fix check can be skipped -- namely nowhere.
    pub fn lat_lon(&self) -> Option<(f64, f64)> {
        if self.gps_fixed {
            Some((LatFac::to_float(self.lat), LonFac::to_float(self.lon)))
        } else {
            None
        }
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
            gps_fixed: self.gps_fixed(),
            lat: self.lat_lon().map(|(lat, _)| lat),
            lon: self.lat_lon().map(|(_, lon)| lon),
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
    /// `None` until the GPS has a fix. `gps_fixed` is anded with this in
    /// `new`, so a builder that sets one and forgets the other still produces
    /// a packet whose validity bit matches its contents.
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
        let (lat, lon) = p.lat_lon().unwrap();
        assert_relative_eq!(lat, 45.5, epsilon = 0.0001);
        assert_relative_eq!(lon, -73.6, epsilon = 0.0001);
    }

    /// With no fix the raw fields still hold the 0.0 filler, so the getter has
    /// to be the thing that refuses -- otherwise the ground station plots the
    /// rocket at Null Island.
    #[test]
    fn no_fix_reports_no_position() {
        let packet = LowPowerTelemetryPacket::new(12, 0, false, None, 8.1, true, 8.2, 27.0);
        assert!(!packet.gps_fixed());
        assert_eq!(packet.lat_lon(), None);

        // And a caller that claims a fix without supplying one does not get to
        // downlink (0, 0) as a position.
        let packet = LowPowerTelemetryPacket::new(12, 4, true, None, 8.1, true, 8.2, 27.0);
        assert!(!packet.gps_fixed());
        assert_eq!(packet.lat_lon(), None);
    }
}
