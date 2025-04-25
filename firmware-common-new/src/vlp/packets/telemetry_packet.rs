use core::cell::{RefCell, RefMut};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex as BlockingMutex};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    can_bus::messages::{
        amp_status::PowerOutputStatus,
        avionics_status::FlightStage,
        node_status::{NodeHealth, NodeMode},
    },
    fixed_point_factory, gps::GPSData,
};

fixed_point_factory!(PayloadVoltageFac, f32, 2.0, 4.5, 0.05);
fixed_point_factory!(PayloadCurrentFac, f32, 0.0, 2.0, 0.1);

#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bits = 66)]
pub struct PayloadTelemetry {
    #[packed_field(element_size_bits = "6", bits = "0..6")]
    eps1_battery1_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "6")]
    eps1_battery2_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "5")]
    eps1_output_3v3_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_3v3_status: PowerOutputStatus,
    #[packed_field(element_size_bits = "5")]
    eps1_output_5v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_5v_status: PowerOutputStatus,
    #[packed_field(element_size_bits = "5")]
    eps1_output_9v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_9v_status: PowerOutputStatus,
    #[packed_field(element_size_bits = "6")]
    eps2_battery1_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "6")]
    eps2_battery2_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "5")]
    eps2_output_3v3_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_3v3_status: PowerOutputStatus,
    #[packed_field(element_size_bits = "5")]
    eps2_output_5v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_5v_status: PowerOutputStatus,
    #[packed_field(element_size_bits = "5")]
    eps2_output_9v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_9v_status: PowerOutputStatus,
}

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);

fixed_point_factory!(BatteryVFac, f32, 5.0, 8.5, 0.02);
fixed_point_factory!(TemperatureFac, f32, -10.0, 85.0, 0.2);
fixed_point_factory!(AltitudeFac, f32, -100.0, 5000.0, 1.0);
fixed_point_factory!(APResidueFac, f32, -1000.0, 1000.0, 1.0);
fixed_point_factory!(AirSpeedFac, f32, -100.0, 400.0, 2.0);
fixed_point_factory!(AirBrakesExtensionInchFac, f32, 0.0, 0.9, 0.04);
fixed_point_factory!(TiltDegFac, f32, -90.0, 90.0, 1.0);
fixed_point_factory!(CdFac, f32, 0.4, 0.85, 0.01);

// 48 byte max size to achieve 0.5Hz with 250khz bandwidth + 12sf + 8cr lora
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "40")]
pub struct TelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    unix_clock_ready: bool,
    num_of_fix_satellites: Integer<u8, packed_bits::Bits<5>>,
    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    vl_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
    #[packed_field(element_size_bits = "9")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "9")]
    vl_stm32_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,

    pyro_main_continuity: bool,
    pyro_drogue_continuity: bool,

    /// above ground level
    #[packed_field(element_size_bits = "13")]
    altitude: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    max_altitude: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    backup_max_altitude: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,
    #[packed_field(element_size_bits = "8")]
    max_air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,
    #[packed_field(element_size_bits = "8")]
    backup_max_air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    tilt_deg: Integer<TiltDegFacBase, packed_bits::Bits<TILT_DEG_FAC_BITS>>,

    flight_core_state: Integer<u8, packed_bits::Bits<3>>,
    backup_flight_core_state: Integer<u8, packed_bits::Bits<3>>,

    amp_online: bool,
    amp_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "8")]
    shared_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out1: PowerOutputStatus,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out2: PowerOutputStatus,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out3: PowerOutputStatus,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out4: PowerOutputStatus,

    main_bulkhead_online: bool,
    main_bulkhead_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "4")]
    main_bulkhead_brightness: u8,

    drogue_bulkhead_online: bool,
    drogue_bulkhead_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "4")]
    drogue_bulkhead_brightness: u8,

    icarus_online: bool,
    icarus_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "5")]
    air_brakes_extention_inch: Integer<
        AirBrakesExtensionInchFacBase,
        packed_bits::Bits<AIR_BRAKES_EXTENSION_INCH_FAC_BITS>,
    >,
    #[packed_field(element_size_bits = "9")]
    air_brakes_servo_temp: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "11")]
    ap_residue: Integer<APResidueFacBase, packed_bits::Bits<A_P_RESIDUE_FAC_BITS>>,
    #[packed_field(element_size_bits = "6")]
    cd: Integer<CdFacBase, packed_bits::Bits<CD_FAC_BITS>>,

    ozys1_online: bool,
    ozys1_rebooted_in_last_5s: bool,

    ozys2_online: bool,
    ozys2_rebooted_in_last_5s: bool,

    payload_activation_online: bool,
    payload_activation_rebooted_in_last_5s: bool,
    payload_alive: bool,

    aero_rust_online: bool,
    aero_rust_rebooted_in_last_5s: bool,

    #[packed_field(element_size_bits = "2", ty = "enum")]
    aero_rust_health: NodeHealth,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    aero_rust_mode: NodeMode,
    #[packed_field(element_size_bits = "12")]
    aero_rust_status: u16,

    #[packed_field(element_size_bits = "66")]
    payload: PayloadTelemetry,
}

impl TelemetryPacket {
    pub fn new(
        nonce: u8,

        unix_clock_ready: bool,
        num_of_fix_satellites: u8,
        lat_lon: Option<(f64, f64)>,

        vl_battery_v: f32,
        air_temperature: f32,
        vl_stm32_temperature: f32,

        pyro_main_continuity: bool,
        pyro_drogue_continuity: bool,

        altitude: f32,
        max_altitude: f32,
        backup_max_altitude: f32,

        air_speed: f32,
        max_air_speed: f32,
        backup_max_air_speed: f32,

        tilt_deg: f32,

        flight_core_state: u8,
        backup_flight_core_state: u8,

        amp_online: bool,
        amp_rebooted_in_last_5s: bool,
        shared_battery_v: f32,
        amp_out1: PowerOutputStatus,
        amp_out2: PowerOutputStatus,
        amp_out3: PowerOutputStatus,
        amp_out4: PowerOutputStatus,

        main_bulkhead_online: bool,
        main_bulkhead_rebooted_in_last_5s: bool,
        main_bulkhead_brightness: u8,

        drogue_bulkhead_online: bool,
        drogue_bulkhead_rebooted_in_last_5s: bool,
        drogue_bulkhead_brightness: u8,

        icarus_online: bool,
        icarus_rebooted_in_last_5s: bool,
        air_brakes_extention_inch: f32,
        air_brakes_servo_temp: f32,
        ap_residue: f32,
        cd: f32,

        ozys1_online: bool,
        ozys1_rebooted_in_last_5s: bool,

        ozys2_online: bool,
        ozys2_rebooted_in_last_5s: bool,

        payload_activation_online: bool,
        payload_activation_rebooted_in_last_5s: bool,
        payload_alive: bool,

        aero_rust_online: bool,
        aero_rust_rebooted_in_last_5s: bool,
        aero_rust_health: NodeHealth,
        aero_rust_mode: NodeMode,
        aero_rust_status: u16,

        payload: PayloadTelemetry,
    ) -> Self {
        Self {
            nonce: nonce.into(),

            unix_clock_ready,
            num_of_fix_satellites: num_of_fix_satellites.into(),
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),

            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),
            vl_stm32_temperature: TemperatureFac::to_fixed_point_capped(vl_stm32_temperature),

            pyro_main_continuity,
            pyro_drogue_continuity,

            altitude: AltitudeFac::to_fixed_point_capped(altitude),
            max_altitude: AltitudeFac::to_fixed_point_capped(max_altitude),
            backup_max_altitude: AltitudeFac::to_fixed_point_capped(backup_max_altitude),

            air_speed: AirSpeedFac::to_fixed_point_capped(air_speed),
            max_air_speed: AirSpeedFac::to_fixed_point_capped(max_air_speed),
            backup_max_air_speed: AirSpeedFac::to_fixed_point_capped(backup_max_air_speed),

            tilt_deg: TiltDegFac::to_fixed_point_capped(tilt_deg),

            flight_core_state: flight_core_state.into(),
            backup_flight_core_state: backup_flight_core_state.into(),

            amp_online,
            amp_rebooted_in_last_5s,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),

            amp_out1,
            amp_out2,
            amp_out3,
            amp_out4,

            main_bulkhead_online,
            main_bulkhead_rebooted_in_last_5s,
            main_bulkhead_brightness,

            drogue_bulkhead_online,
            drogue_bulkhead_rebooted_in_last_5s,
            drogue_bulkhead_brightness,

            icarus_online,
            icarus_rebooted_in_last_5s,
            air_brakes_extention_inch: AirBrakesExtensionInchFac::to_fixed_point_capped(
                air_brakes_extention_inch,
            ),
            air_brakes_servo_temp: TemperatureFac::to_fixed_point_capped(air_brakes_servo_temp),
            ap_residue: APResidueFac::to_fixed_point_capped(ap_residue),
            cd: CdFac::to_fixed_point_capped(cd),

            ozys1_online,
            ozys1_rebooted_in_last_5s,

            ozys2_online,
            ozys2_rebooted_in_last_5s,

            payload_activation_online,
            payload_activation_rebooted_in_last_5s,
            payload_alive,

            aero_rust_online,
            aero_rust_rebooted_in_last_5s,
            aero_rust_health,
            aero_rust_mode,
            aero_rust_status,

            payload,
        }
    }

    pub fn unix_clock_ready(&self) -> bool {
        self.unix_clock_ready
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites.into()
    }

    pub fn lat_lon(&self) -> (f64, f64) {
        (LatFac::to_float(self.lat), LonFac::to_float(self.lon))
    }

    pub fn vl_battery_v(&self) -> f32 {
        BatteryVFac::to_float(self.vl_battery_v)
    }

    pub fn air_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.air_temperature)
    }

    pub fn vl_stm32_temperature(&self) -> f32 {
        TemperatureFac::to_float(self.vl_stm32_temperature)
    }

    pub fn pyro_main_continuity(&self) -> bool {
        self.pyro_main_continuity
    }

    pub fn pyro_drogue_continuity(&self) -> bool {
        self.pyro_drogue_continuity
    }

    pub fn altitude(&self) -> f32 {
        AltitudeFac::to_float(self.altitude)
    }

    pub fn max_altitude(&self) -> f32 {
        AltitudeFac::to_float(self.max_altitude)
    }

    pub fn backup_max_altitude(&self) -> f32 {
        AltitudeFac::to_float(self.backup_max_altitude)
    }

    pub fn air_speed(&self) -> f32 {
        AirSpeedFac::to_float(self.air_speed)
    }

    pub fn max_air_speed(&self) -> f32 {
        AirSpeedFac::to_float(self.max_air_speed)
    }

    pub fn backup_max_air_speed(&self) -> f32 {
        AirSpeedFac::to_float(self.backup_max_air_speed)
    }

    pub fn tilt_deg(&self) -> f32 {
        TiltDegFac::to_float(self.tilt_deg)
    }

    pub fn flight_core_state(&self) -> u8 {
        self.flight_core_state.into()
    }

    pub fn backup_flight_core_state(&self) -> u8 {
        self.backup_flight_core_state.into()
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

    pub fn amp_out1(&self) -> PowerOutputStatus {
        self.amp_out1
    }

    pub fn amp_out2(&self) -> PowerOutputStatus {
        self.amp_out2
    }

    pub fn amp_out3(&self) -> PowerOutputStatus {
        self.amp_out3
    }

    pub fn amp_out4(&self) -> PowerOutputStatus {
        self.amp_out4
    }

    pub fn main_bulkhead_online(&self) -> bool {
        self.main_bulkhead_online
    }

    pub fn main_bulkhead_rebooted_in_last_5s(&self) -> bool {
        self.main_bulkhead_rebooted_in_last_5s
    }

    pub fn main_bulkhead_brightness(&self) -> u8 {
        self.main_bulkhead_brightness
    }

    pub fn drogue_bulkhead_online(&self) -> bool {
        self.drogue_bulkhead_online
    }

    pub fn drogue_bulkhead_rebooted_in_last_5s(&self) -> bool {
        self.drogue_bulkhead_rebooted_in_last_5s
    }

    pub fn drogue_bulkhead_brightness(&self) -> u8 {
        self.drogue_bulkhead_brightness
    }

    pub fn icarus_online(&self) -> bool {
        self.icarus_online
    }

    pub fn icarus_rebooted_in_last_5s(&self) -> bool {
        self.icarus_rebooted_in_last_5s
    }

    pub fn air_brakes_extention_inch(&self) -> f32 {
        AirBrakesExtensionInchFac::to_float(self.air_brakes_extention_inch)
    }

    pub fn air_brakes_servo_temp(&self) -> f32 {
        TemperatureFac::to_float(self.air_brakes_servo_temp)
    }

    pub fn ap_residue(&self) -> f32 {
        APResidueFac::to_float(self.ap_residue)
    }

    pub fn cd(&self) -> f32 {
        CdFac::to_float(self.cd)
    }

    pub fn ozys1_online(&self) -> bool {
        self.ozys1_online
    }

    pub fn ozys1_rebooted_in_last_5s(&self) -> bool {
        self.ozys1_rebooted_in_last_5s
    }

    pub fn ozys2_online(&self) -> bool {
        self.ozys2_online
    }

    pub fn ozys2_rebooted_in_last_5s(&self) -> bool {
        self.ozys2_rebooted_in_last_5s
    }

    pub fn payload_activation_online(&self) -> bool {
        self.payload_activation_online
    }

    pub fn payload_activation_rebooted_in_last_5s(&self) -> bool {
        self.payload_activation_rebooted_in_last_5s
    }

    pub fn payload_alive(&self) -> bool {
        self.payload_alive
    }

    pub fn aero_rust_online(&self) -> bool {
        self.aero_rust_online
    }

    pub fn aero_rust_rebooted_in_last_5s(&self) -> bool {
        self.aero_rust_rebooted_in_last_5s
    }

    pub fn aero_rust_health(&self) -> NodeHealth {
        self.aero_rust_health
    }

    pub fn aero_rust_mode(&self) -> NodeMode {
        self.aero_rust_mode
    }

    pub fn aero_rust_status(&self) -> u16 {
        self.aero_rust_status
    }

    pub fn payload(&self) -> &PayloadTelemetry {
        &self.payload
    }
}

pub struct TelemetryPacketBuilderState {
    nonce: u8,

    pub gps_location: Option<GPSData>,

    pub vl_battery_v: f32,
    pub air_temperature: f32,
    pub vl_stm32_temperature: f32,

    pub pyro_main_continuity: bool,
    pub pyro_drogue_continuity: bool,

    pub altitude: f32,
    max_altitude: f32,
    pub backup_altitude: f32,
    backup_max_altitude: f32,
    
    pub air_speed: f32,
    max_air_speed: f32,
    pub backup_air_speed: f32,
    backup_max_air_speed: f32,

    pub tilt_deg: f32,

    pub flight_core_state: u8,
    pub backup_flight_core_state:u8,

    pub amp_online: bool,
    pub amp_uptime_s: u32,
    pub shared_battery_v: f32,
    pub amp_out1: PowerOutputStatus,
    pub amp_out2: PowerOutputStatus,
    pub amp_out3: PowerOutputStatus,
    pub amp_out4: PowerOutputStatus,
    
    pub main_bulkhead_online: bool,
}

// pub struct TelemetryPacketBuilder<'a, K: Clock> {
//     unix_clock: UnixClock<'a, K>,
//     state: BlockingMutex<NoopRawMutex, RefCell<TelemetryPacketBuilderState>>,
// }

// impl<'a, K: Clock> TelemetryPacketBuilder<'a, K> {
//     pub fn new(unix_clock: UnixClock<'a, K>) -> Self {
//         Self {
//             unix_clock,
//             state: BlockingMutex::new(RefCell::new(TelemetryPacketBuilderState {
//                 gps_location: None,
//                 battery_v: 0.0,
//                 temperature: 0.0,
//                 altitude: 0.0,
//                 max_altitude: 0.0,
//                 backup_altitude: 0.0,
//                 backup_max_altitude: 0.0,
//                 air_speed: 0.0,
//                 max_air_speed: 0.0,
//                 backup_air_speed: 0.0,
//                 backup_max_air_speed: 0.0,
//                 hardware_armed: false,
//                 software_armed: false,
//                 pyro_main_continuity: false,
//                 pyro_drogue_continuity: false,
//                 flight_core_state: FlightCoreState::DisArmed,
//                 backup_flight_core_state: FlightCoreState::DisArmed,
//                 disk_free_space: 0,
//                 drogue_deployed: false,
//                 main_deployed: false,
//             })),
//         }
//     }

//     pub fn create_packet(&self) -> TelemetryPacket {
//         self.state.lock(|state| {
//             let state = state.borrow();

//             TelemetryPacket::new(
//                 self.unix_clock.ready(),
//                 self.unix_clock.now_ms(),
//                 state
//                     .gps_location
//                     .as_ref()
//                     .map_or(0, |l| l.num_of_fix_satellites),
//                 state.gps_location.as_ref().map(|l| l.lat_lon).flatten(),
//                 state.battery_v,
//                 state.temperature,
//                 state.hardware_armed,
//                 state.software_armed,
//                 state.disk_free_space,
//                 state.pyro_main_continuity,
//                 state.pyro_drogue_continuity,
//                 state.altitude,
//                 state.max_altitude,
//                 state.backup_max_altitude,
//                 state.air_speed,
//                 state.max_air_speed,
//                 state.backup_max_air_speed,
//                 state.flight_core_state,
//                 state.backup_flight_core_state,
//                 state.drogue_deployed,
//                 state.main_deployed,
//             )
//         })
//     }

//     pub fn update<U>(&self, update_fn: U)
//     where
//         U: FnOnce(&mut RefMut<TelemetryPacketBuilderState>) -> (),
//     {
//         self.state.lock(|state| {
//             let mut state = state.borrow_mut();
//             update_fn(&mut state);
//             state.max_altitude = state.altitude.max(state.max_altitude);
//             state.max_air_speed = state.air_speed.max(state.max_air_speed);
//             state.backup_max_altitude = state.backup_altitude.max(state.backup_max_altitude);
//             state.backup_max_air_speed = state.backup_air_speed.max(state.backup_max_air_speed);
//         })
//     }
// }

// #[cfg(test)]
// mod test {
//     use super::*;

//     #[test]
//     fn print_telemetry_packet_length() {
//         println!(
//             "Telemetry Packet Struct Length: {}",
//             size_of::<TelemetryPacket>() * 8
//         );
//         println!("Telemetry Packet Length: {}", TelemetryPacket::len_bits());
//     }
// }
