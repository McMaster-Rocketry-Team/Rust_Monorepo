use core::cell::{RefCell, RefMut};

use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::fixed_point_factory;

use super::VLPDownlinkPacket;

fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);
fixed_point_factory!(TemperatureFac, f32, 0.0, 85.0, 0.2);

#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
pub struct LowPowerTelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    #[packed_field(element_size_bits = "5")]
    num_of_fix_satellites: u8,
    pub gps_fixed: bool,

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
        vl_battery_v: f32,
        amp_online: bool,
        shared_battery_v: f32,
        air_temperature: f32,
    ) -> Self {
        Self {
            nonce: nonce.into(),
            num_of_fix_satellites,
            gps_fixed,
            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            amp_online,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
        }
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites
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
