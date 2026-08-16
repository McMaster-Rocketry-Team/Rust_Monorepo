use core::cell::{RefCell, RefMut};

use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{can_bus::messages::amp_status::PowerOutputStatus, fixed_point_factory};

use super::VLPDownlinkPacket;

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);
fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);

#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
// 88 bits = exactly 11 bytes, no spare. Same content-width as
// LowPowerTelemetryPacket, which is why both are 11. The last free bit went to
// `lat_lon_valid`; this packet is sized to its contents and there is nothing
// left to spend, so any new field costs a twelfth byte.
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "11")]
pub struct LandedTelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    /// Whether `lat` / `lon` hold a position at all. This is the packet the
    /// recovery team navigates by, so a fix that was never acquired has to
    /// read as "no position" rather than as the (0, 0) the empty fields would
    /// otherwise decode to. It is a separate question from
    /// `num_of_fix_satellites`, which comes from a different NMEA field and
    /// can be nonzero while the receiver is still tracking without a solution.
    lat_lon_valid: bool,
    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,

    #[packed_field(element_size_bits = "10")]
    battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,

    amp_online: bool,
    amp_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "10")]
    shared_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
    amp_out1_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out1: PowerOutputStatus,
    amp_out2_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out2: PowerOutputStatus,
    amp_out3_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out3: PowerOutputStatus,
    // amp_out4_overwrote: bool,
    // #[packed_field(element_size_bits = "2", ty = "enum")]
    // amp_out4: PowerOutputStatus,
}

impl LandedTelemetryPacket {
    pub fn new(
        nonce: u8,
        lat_lon: Option<(f64, f64)>,
        num_of_fix_satellites: u8,
        battery_v: f32,
        amp_online: bool,
        amp_rebooted_in_last_5s: bool,
        shared_battery_v: f32,
        amp_out1_overwrote: bool,
        amp_out1: PowerOutputStatus,
        amp_out2_overwrote: bool,
        amp_out2: PowerOutputStatus,
        amp_out3_overwrote: bool,
        amp_out3: PowerOutputStatus,
        // amp_out4_overwrote: bool,
        // amp_out4: PowerOutputStatus,
    ) -> Self {
        Self {
            nonce: nonce.into(),
            lat_lon_valid: lat_lon.is_some(),
            // The 0.0 filler is never read back: `lat_lon` refuses to decode
            // unless `lat_lon_valid` is set.
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),
            num_of_fix_satellites,
            battery_v: BatteryVFac::to_fixed_point_capped(battery_v),
            amp_online,
            amp_rebooted_in_last_5s,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),
            amp_out1_overwrote,
            amp_out1,
            amp_out2_overwrote,
            amp_out2,
            amp_out3_overwrote,
            amp_out3,
            // amp_out4_overwrote,
            // amp_out4,
        }
    }

    /// `None` when the rocket landed without ever having had a fix, or lost it
    /// before touchdown. One getter rather than a `lat()` and a `lon()`: half a
    /// coordinate is not a place, and a single `Option` leaves no way to read
    /// the position without first answering whether there is one.
    pub fn lat_lon(&self) -> Option<(f64, f64)> {
        if self.lat_lon_valid {
            Some((LatFac::to_float(self.lat), LonFac::to_float(self.lon)))
        } else {
            None
        }
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
    }

    pub fn battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.battery_v)
    }

    pub fn amp_online(&self) -> bool {
        self.amp_online
    }

    pub fn amp_rebooted_in_last_5s(&self) -> bool {
        self.amp_rebooted_in_last_5s
    }

    pub fn shared_battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.shared_battery_v)
    }

    pub fn amp_out1_overwrote(&self) -> bool {
        self.amp_out1_overwrote
    }

    pub fn amp_out1(&self) -> PowerOutputStatus {
        self.amp_out1
    }

    pub fn amp_out2_overwrote(&self) -> bool {
        self.amp_out2_overwrote
    }

    pub fn amp_out2(&self) -> PowerOutputStatus {
        self.amp_out2
    }

    pub fn amp_out3_overwrote(&self) -> bool {
        self.amp_out3_overwrote
    }

    pub fn amp_out3(&self) -> PowerOutputStatus {
        self.amp_out3
    }

    // pub fn amp_out4_overwrote(&self) -> bool {
    //     self.amp_out4_overwrote
    // }

    // pub fn amp_out4(&self) -> PowerOutputStatus {
    //     self.amp_out4
    // }

    /// `lat` / `lon` are `null` when there is no fix, rather than the (0, 0)
    /// the raw fields hold. The key set is unchanged.
    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            lat: self.lat_lon().map(|(lat, _)| lat),
            lon: self.lat_lon().map(|(_, lon)| lon),
            num_of_fix_satellites: self.num_of_fix_satellites(),
            battery_v: self.battery_v(),
            amp_online: self.amp_online(),
            amp_rebooted_in_last_5s: self.amp_rebooted_in_last_5s(),
            shared_battery_v: self.shared_battery_v(),
            amp_out1_overwrote: self.amp_out1_overwrote(),
            amp_out1: format!("{:?}", self.amp_out1()),
            amp_out2_overwrote: self.amp_out2_overwrote(),
            amp_out2: format!("{:?}", self.amp_out2()),
            amp_out3_overwrote: self.amp_out3_overwrote(),
            amp_out3: format!("{:?}", self.amp_out3()),
            // amp_out4_overwrote: self.amp_out4_overwrote(),
            // amp_out4: format!("{:?}", self.amp_out4()),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for LandedTelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "LandedTelemetryPacket")
    }
}

impl Into<VLPDownlinkPacket> for LandedTelemetryPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::LandedTelemetry(self)
    }
}

pub struct LandedTelemetryPacketBuilderState {
    nonce: u8,
    /// `None` until the GPS has a fix, and `None` again if it loses one.
    pub lat_lon: Option<(f64, f64)>,
    pub num_of_fix_satellites: u8,
    pub battery_v: f32,
    pub amp_online: bool,
    pub amp_rebooted_in_last_5s: bool,
    pub shared_battery_v: f32,
    pub amp_out1_overwrote: bool,
    pub amp_out1: PowerOutputStatus,
    pub amp_out2_overwrote: bool,
    pub amp_out2: PowerOutputStatus,
    pub amp_out3_overwrote: bool,
    pub amp_out3: PowerOutputStatus,
    // pub amp_out4_overwrote: bool,
    // pub amp_out4: PowerOutputStatus,
}

pub struct LandedTelemetryPacketBuilder<M: RawMutex> {
    state: BlockingMutex<M, RefCell<LandedTelemetryPacketBuilderState>>,
}

impl<M: RawMutex> LandedTelemetryPacketBuilder<M> {
    pub fn new() -> Self {
        Self {
            state: BlockingMutex::new(RefCell::new(LandedTelemetryPacketBuilderState {
                nonce: 0,
                lat_lon: None,
                num_of_fix_satellites: 0,
                battery_v: 0.0,
                amp_online: false,
                amp_rebooted_in_last_5s: false,
                shared_battery_v: 0.0,
                amp_out1_overwrote: false,
                amp_out1: PowerOutputStatus::Disabled,
                amp_out2_overwrote: false,
                amp_out2: PowerOutputStatus::Disabled,
                amp_out3_overwrote: false,
                amp_out3: PowerOutputStatus::Disabled,
                // amp_out4_overwrote: false,
                // amp_out4: PowerOutputStatus::Disabled,
            })),
        }
    }

    pub fn create_packet(&self) -> LandedTelemetryPacket {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.nonce += 1;
            if state.nonce > 15 {
                state.nonce = 0;
            }
            LandedTelemetryPacket::new(
                state.nonce,
                state.lat_lon,
                state.num_of_fix_satellites,
                state.battery_v,
                state.amp_online,
                state.amp_rebooted_in_last_5s,
                state.shared_battery_v,
                state.amp_out1_overwrote,
                state.amp_out1,
                state.amp_out2_overwrote,
                state.amp_out2,
                state.amp_out3_overwrote,
                state.amp_out3,
                // state.amp_out4_overwrote,
                // state.amp_out4,
            )
        })
    }

    pub fn update<U>(&self, update_fn: U)
    where
        U: FnOnce(&mut RefMut<LandedTelemetryPacketBuilderState>) -> (),
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

    fn packet(lat_lon: Option<(f64, f64)>) -> LandedTelemetryPacket {
        LandedTelemetryPacket::new(
            12,
            lat_lon,
            9,
            7.9,
            true,
            false,
            8.2,
            false,
            PowerOutputStatus::PowerGood,
            true,
            PowerOutputStatus::PowerBad,
            false,
            PowerOutputStatus::Disabled,
        )
    }

    fn round_trip(packet: LandedTelemetryPacket) -> LandedTelemetryPacket {
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);
        // 1 byte packet type + the 11 byte packed struct. The `lat_lon_valid`
        // bit took the last spare bit, so this stays 12.
        assert_eq!(len, 12);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();
        assert_eq!(deserialized_packet, packet);

        let VLPDownlinkPacket::LandedTelemetry(p) = deserialized_packet else {
            unreachable!()
        };
        p
    }

    #[test]
    fn test_serialize_deserialize() {
        let p = round_trip(packet(Some((45.5, -73.6))));

        let (lat, lon) = p.lat_lon().unwrap();
        assert_relative_eq!(lat, 45.5, epsilon = 0.0001);
        assert_relative_eq!(lon, -73.6, epsilon = 0.0001);
        assert_eq!(p.num_of_fix_satellites(), 9);
        assert_relative_eq!(p.battery_v(), 7.9, epsilon = 0.01);
        assert_eq!(p.amp_out2(), PowerOutputStatus::PowerBad);
    }

    /// The packet a rocket that landed without a fix sends. It must say so:
    /// this is the one the recovery team walks in on, and (0, 0) is a place.
    #[test]
    fn no_fix_reports_no_position() {
        let p = round_trip(packet(None));

        assert_eq!(p.lat_lon(), None);
        // The rest of the packet is still worth having.
        assert_relative_eq!(p.battery_v(), 7.9, epsilon = 0.01);
        assert!(p.amp_online());
    }
}
