use core::cell::{RefCell, RefMut};
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use micromath::F32Ext;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    can_bus::messages::{
        amp_status::PowerOutputStatus,
        avionics_status::FlightStage,
        node_status::{NodeHealth, NodeMode},
    },
    fixed_point_factory,
    gps::GPSData,
};

use super::VLPDownlinkPacket;

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);

fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);
fixed_point_factory!(TemperatureFac, f32, -10.0, 85.0, 0.2);
fixed_point_factory!(AltitudeFac, f32, -100.0, 6000.0, 1.0);
fixed_point_factory!(APResidueFac, f32, -1000.0, 1000.0, 1.0);
fixed_point_factory!(AirSpeedFac, f32, -100.0, 400.0, 2.0);
fixed_point_factory!(AirBrakesExtensionPercentFac, f32, 0.0, 1.0, 0.04);
fixed_point_factory!(TiltDegFac, f32, -90.0, 90.0, 1.0);
fixed_point_factory!(CdFac, f32, 0.4, 0.85, 0.01);

fixed_point_factory!(PayloadVoltageFac, f32, 2.0, 4.5, 0.05);
fixed_point_factory!(PayloadCurrentFac, f32, 0.0, 2.0, 0.1);
fixed_point_factory!(PayloadTemperatureFac, f32, 10.0, 85.0, 1.0);

// 48 byte max size to achieve 0.5Hz with 250khz bandwidth + 12sf + 8cr lora
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "46")]
pub struct TelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    unix_clock_ready: bool,
    num_of_fix_satellites: Integer<u8, packed_bits::Bits<5>>,
    #[packed_field(element_size_bits = "23")]
    lat: Integer<LatFacBase, packed_bits::Bits<LAT_FAC_BITS>>,
    #[packed_field(element_size_bits = "24")]
    lon: Integer<LonFacBase, packed_bits::Bits<LON_FAC_BITS>>,

    #[packed_field(element_size_bits = "10")]
    vl_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
    #[packed_field(element_size_bits = "9")]
    air_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "9")]
    vl_stm32_temperature: Integer<TemperatureFacBase, packed_bits::Bits<TEMPERATURE_FAC_BITS>>,

    pyro_main_continuity: bool,
    pyro_drogue_continuity: bool,

    #[packed_field(element_size_bits = "13")]
    altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    max_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    backup_max_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,
    #[packed_field(element_size_bits = "8")]
    max_air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,
    #[packed_field(element_size_bits = "8")]
    backup_max_air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    tilt_deg: Integer<TiltDegFacBase, packed_bits::Bits<TILT_DEG_FAC_BITS>>,

    #[packed_field(element_size_bits = "3", ty = "enum")]
    flight_stage: FlightStage,
    #[packed_field(element_size_bits = "3", ty = "enum")]
    backup_flight_stage: FlightStage,

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
    amp_out4_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    amp_out4: PowerOutputStatus,

    main_bulkhead_online: bool,
    main_bulkhead_rebooted_in_last_5s: bool,
    main_bulkhead_brightness: u8,

    drogue_bulkhead_online: bool,
    drogue_bulkhead_rebooted_in_last_5s: bool,
    drogue_bulkhead_brightness: u8,

    icarus_online: bool,
    icarus_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "5")]
    air_brakes_commanded_extension_percentage: Integer<
        AirBrakesExtensionPercentFacBase,
        packed_bits::Bits<AIR_BRAKES_EXTENSION_PERCENT_FAC_BITS>,
    >,
    #[packed_field(element_size_bits = "5")]
    air_brakes_actual_extension_percentage: Integer<
        AirBrakesExtensionPercentFacBase,
        packed_bits::Bits<AIR_BRAKES_EXTENSION_PERCENT_FAC_BITS>,
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

    aero_rust_online: bool,
    aero_rust_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    aero_rust_health: NodeHealth,

    payload_activation_pcb_online: bool,
    payload_activation_pcb_rebooted_in_last_5s: bool,

    rocket_wifi_online: bool,
    rocket_wifi_rebooted_in_last_5s: bool,

    eps1_online: bool,
    eps1_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "6")]
    eps1_battery1_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    eps1_battery1_temperature:
        Integer<PayloadTemperatureFacBase, packed_bits::Bits<PAYLOAD_TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "6")]
    eps1_battery2_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    eps1_battery2_temperature:
        Integer<PayloadTemperatureFacBase, packed_bits::Bits<PAYLOAD_TEMPERATURE_FAC_BITS>>,

    #[packed_field(element_size_bits = "5")]
    eps1_output_3v3_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps1_output_3v3_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_3v3_status: PowerOutputStatus,

    #[packed_field(element_size_bits = "5")]
    eps1_output_5v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps1_output_5v_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_5v_status: PowerOutputStatus,

    #[packed_field(element_size_bits = "5")]
    eps1_output_9v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps1_output_9v_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps1_output_9v_status: PowerOutputStatus,

    eps2_online: bool,
    eps2_rebooted_in_last_5s: bool,
    #[packed_field(element_size_bits = "6")]
    eps2_battery1_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    eps2_battery1_temperature:
        Integer<PayloadTemperatureFacBase, packed_bits::Bits<PAYLOAD_TEMPERATURE_FAC_BITS>>,
    #[packed_field(element_size_bits = "6")]
    eps2_battery2_v: Integer<PayloadVoltageFacBase, packed_bits::Bits<PAYLOAD_VOLTAGE_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    eps2_battery2_temperature:
        Integer<PayloadTemperatureFacBase, packed_bits::Bits<PAYLOAD_TEMPERATURE_FAC_BITS>>,

    #[packed_field(element_size_bits = "5")]
    eps2_output_3v3_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps2_output_3v3_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_3v3_status: PowerOutputStatus,

    #[packed_field(element_size_bits = "5")]
    eps2_output_5v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps2_output_5v_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_5v_status: PowerOutputStatus,

    #[packed_field(element_size_bits = "5")]
    eps2_output_9v_current:
        Integer<PayloadCurrentFacBase, packed_bits::Bits<PAYLOAD_CURRENT_FAC_BITS>>,
    eps2_output_9v_overwrote: bool,
    #[packed_field(element_size_bits = "2", ty = "enum")]
    eps2_output_9v_status: PowerOutputStatus,
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

        altitude_agl: f32,
        max_altitude_agl: f32,
        backup_max_altitude_agl: f32,

        air_speed: f32,
        max_air_speed: f32,
        backup_max_air_speed: f32,

        tilt_deg: f32,

        flight_stage: FlightStage,
        backup_flight_stage: FlightStage,

        amp_online: bool,
        amp_rebooted_in_last_5s: bool,
        shared_battery_v: f32,
        amp_out1_overwrote: bool,
        amp_out1: PowerOutputStatus,
        amp_out2_overwrote: bool,
        amp_out2: PowerOutputStatus,
        amp_out3_overwrote: bool,
        amp_out3: PowerOutputStatus,
        amp_out4_overwrote: bool,
        amp_out4: PowerOutputStatus,

        main_bulkhead_online: bool,
        main_bulkhead_rebooted_in_last_5s: bool,
        main_bulkhead_brightness: f32,

        drogue_bulkhead_online: bool,
        drogue_bulkhead_rebooted_in_last_5s: bool,
        drogue_bulkhead_brightness: f32,

        icarus_online: bool,
        icarus_rebooted_in_last_5s: bool,
        air_brakes_commanded_extension_percentage: f32,
        air_brakes_actual_extension_percentage: f32,
        air_brakes_servo_temp: f32,
        ap_residue: f32,
        cd: f32,

        ozys1_online: bool,
        ozys1_rebooted_in_last_5s: bool,

        ozys2_online: bool,
        ozys2_rebooted_in_last_5s: bool,

        aero_rust_online: bool,
        aero_rust_rebooted_in_last_5s: bool,
        aero_rust_health: NodeHealth,

        payload_activation_pcb_online: bool,
        payload_activation_pcb_rebooted_in_last_5s: bool,

        rocket_wifi_online: bool,
        rocket_wifi_rebooted_in_last_5s: bool,

        eps1_online: bool,
        eps1_rebooted_in_last_5s: bool,
        eps1_battery1_v: f32,
        eps1_battery1_temperature: f32,
        eps1_battery2_v: f32,
        eps1_battery2_temperature: f32,
        eps1_output_3v3_current: f32,
        eps1_output_3v3_overwrote: bool,
        eps1_output_3v3_status: PowerOutputStatus,
        eps1_output_5v_current: f32,
        eps1_output_5v_overwrote: bool,
        eps1_output_5v_status: PowerOutputStatus,
        eps1_output_9v_current: f32,
        eps1_output_9v_overwrote: bool,
        eps1_output_9v_status: PowerOutputStatus,

        eps2_online: bool,
        eps2_rebooted_in_last_5s: bool,
        eps2_battery1_v: f32,
        eps2_battery1_temperature: f32,
        eps2_battery2_v: f32,
        eps2_battery2_temperature: f32,
        eps2_output_3v3_current: f32,
        eps2_output_3v3_overwrote: bool,
        eps2_output_3v3_status: PowerOutputStatus,
        eps2_output_5v_current: f32,
        eps2_output_5v_overwrote: bool,
        eps2_output_5v_status: PowerOutputStatus,
        eps2_output_9v_current: f32,
        eps2_output_9v_overwrote: bool,
        eps2_output_9v_status: PowerOutputStatus,
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

            altitude_agl: AltitudeFac::to_fixed_point_capped(altitude_agl),
            max_altitude_agl: AltitudeFac::to_fixed_point_capped(max_altitude_agl),
            backup_max_altitude_agl: AltitudeFac::to_fixed_point_capped(backup_max_altitude_agl),

            air_speed: AirSpeedFac::to_fixed_point_capped(air_speed),
            max_air_speed: AirSpeedFac::to_fixed_point_capped(max_air_speed),
            backup_max_air_speed: AirSpeedFac::to_fixed_point_capped(backup_max_air_speed),

            tilt_deg: TiltDegFac::to_fixed_point_capped(tilt_deg),

            flight_stage: flight_stage.into(),
            backup_flight_stage: backup_flight_stage.into(),

            amp_online,
            amp_rebooted_in_last_5s,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),

            amp_out1_overwrote,
            amp_out1,
            amp_out2_overwrote,
            amp_out2,
            amp_out3_overwrote,
            amp_out3,
            amp_out4_overwrote,
            amp_out4,

            main_bulkhead_online,
            main_bulkhead_rebooted_in_last_5s,
            main_bulkhead_brightness: TelemetryPacket::encode_brightness_lux(
                main_bulkhead_brightness,
            ),

            drogue_bulkhead_online,
            drogue_bulkhead_rebooted_in_last_5s,
            drogue_bulkhead_brightness: TelemetryPacket::encode_brightness_lux(
                drogue_bulkhead_brightness,
            ),

            icarus_online,
            icarus_rebooted_in_last_5s,
            air_brakes_commanded_extension_percentage:
                AirBrakesExtensionPercentFac::to_fixed_point_capped(
                    air_brakes_commanded_extension_percentage,
                ),
            air_brakes_actual_extension_percentage:
                AirBrakesExtensionPercentFac::to_fixed_point_capped(
                    air_brakes_actual_extension_percentage,
                ),
            air_brakes_servo_temp: TemperatureFac::to_fixed_point_capped(air_brakes_servo_temp),
            ap_residue: APResidueFac::to_fixed_point_capped(ap_residue),
            cd: CdFac::to_fixed_point_capped(cd),

            ozys1_online,
            ozys1_rebooted_in_last_5s,

            ozys2_online,
            ozys2_rebooted_in_last_5s,

            aero_rust_online,
            aero_rust_rebooted_in_last_5s,
            aero_rust_health,

            payload_activation_pcb_online,
            payload_activation_pcb_rebooted_in_last_5s,

            rocket_wifi_online,
            rocket_wifi_rebooted_in_last_5s,

            eps1_online,
            eps1_rebooted_in_last_5s,
            eps1_battery1_v: PayloadVoltageFac::to_fixed_point_capped(eps1_battery1_v),
            eps1_battery1_temperature: PayloadTemperatureFac::to_fixed_point_capped(
                eps1_battery1_temperature,
            ),
            eps1_battery2_v: PayloadVoltageFac::to_fixed_point_capped(eps1_battery2_v),
            eps1_battery2_temperature: PayloadTemperatureFac::to_fixed_point_capped(
                eps1_battery2_temperature,
            ),
            eps1_output_3v3_current: PayloadCurrentFac::to_fixed_point_capped(
                eps1_output_3v3_current,
            ),
            eps1_output_3v3_overwrote,
            eps1_output_3v3_status,
            eps1_output_5v_current: PayloadCurrentFac::to_fixed_point_capped(
                eps1_output_5v_current,
            ),
            eps1_output_5v_overwrote,
            eps1_output_5v_status,
            eps1_output_9v_current: PayloadCurrentFac::to_fixed_point_capped(
                eps1_output_9v_current,
            ),
            eps1_output_9v_overwrote,
            eps1_output_9v_status,

            eps2_online,
            eps2_rebooted_in_last_5s,
            eps2_battery1_v: PayloadVoltageFac::to_fixed_point_capped(eps2_battery1_v),
            eps2_battery1_temperature: PayloadTemperatureFac::to_fixed_point_capped(
                eps2_battery1_temperature,
            ),
            eps2_battery2_v: PayloadVoltageFac::to_fixed_point_capped(eps2_battery2_v),
            eps2_battery2_temperature: PayloadTemperatureFac::to_fixed_point_capped(
                eps2_battery2_temperature,
            ),
            eps2_output_3v3_current: PayloadCurrentFac::to_fixed_point_capped(
                eps2_output_3v3_current,
            ),
            eps2_output_3v3_overwrote,
            eps2_output_3v3_status,
            eps2_output_5v_current: PayloadCurrentFac::to_fixed_point_capped(
                eps2_output_5v_current,
            ),
            eps2_output_5v_overwrote,
            eps2_output_5v_status,
            eps2_output_9v_current: PayloadCurrentFac::to_fixed_point_capped(
                eps2_output_9v_current,
            ),
            eps2_output_9v_overwrote,
            eps2_output_9v_status,
        }
    }

    fn encode_brightness_lux(brightness_lux: f32) -> u8 {
        F32Ext::round(F32Ext::log(brightness_lux, 1.04f32)) as u8
    }

    fn decode_brightness_lux(brightness_lux: u8) -> f32 {
        F32Ext::powf(1.04, brightness_lux as f32)
    }

    pub fn unix_clock_ready(&self) -> bool {
        self.unix_clock_ready
    }

    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites.into()
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

    pub fn altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.altitude_agl)
    }

    pub fn max_altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.max_altitude_agl)
    }

    pub fn backup_max_altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.backup_max_altitude_agl)
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

    pub fn flight_stage(&self) -> FlightStage {
        self.flight_stage
    }

    pub fn backup_flight_stage(&self) -> FlightStage {
        self.backup_flight_stage
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

    pub fn amp_out4_overwrote(&self) -> bool {
        self.amp_out4_overwrote
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

    pub fn main_bulkhead_brightness_lux(&self) -> f32 {
        Self::decode_brightness_lux(self.main_bulkhead_brightness)
    }

    pub fn drogue_bulkhead_online(&self) -> bool {
        self.drogue_bulkhead_online
    }

    pub fn drogue_bulkhead_rebooted_in_last_5s(&self) -> bool {
        self.drogue_bulkhead_rebooted_in_last_5s
    }

    pub fn drogue_bulkhead_brightness_lux(&self) -> f32 {
        Self::decode_brightness_lux(self.drogue_bulkhead_brightness)
    }

    pub fn icarus_online(&self) -> bool {
        self.icarus_online
    }

    pub fn icarus_rebooted_in_last_5s(&self) -> bool {
        self.icarus_rebooted_in_last_5s
    }

    pub fn air_brakes_commanded_extension_percentage(&self) -> f32 {
        AirBrakesExtensionPercentFac::to_float(self.air_brakes_commanded_extension_percentage)
    }

    pub fn air_brakes_actual_extension_percentage(&self) -> f32 {
        AirBrakesExtensionPercentFac::to_float(self.air_brakes_actual_extension_percentage)
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

    pub fn aero_rust_online(&self) -> bool {
        self.aero_rust_online
    }

    pub fn aero_rust_rebooted_in_last_5s(&self) -> bool {
        self.aero_rust_rebooted_in_last_5s
    }

    pub fn aero_rust_health(&self) -> NodeHealth {
        self.aero_rust_health
    }

    pub fn payload_activation_pcb_online(&self) -> bool {
        self.payload_activation_pcb_online
    }

    pub fn payload_activation_pcb_rebooted_in_last_5s(&self) -> bool {
        self.payload_activation_pcb_rebooted_in_last_5s
    }

    pub fn rocket_wifi_online(&self) -> bool {
        self.rocket_wifi_online
    }

    pub fn rocket_wifi_rebooted_in_last_5s(&self) -> bool {
        self.rocket_wifi_rebooted_in_last_5s
    }

    pub fn eps1_online(&self) -> bool {
        self.eps1_online
    }

    pub fn eps1_rebooted_in_last_5s(&self) -> bool {
        self.eps1_rebooted_in_last_5s
    }

    pub fn eps1_battery1_v(&self) -> f32 {
        PayloadVoltageFac::to_float(self.eps1_battery1_v)
    }

    pub fn eps1_battery1_temperature(&self) -> f32 {
        PayloadTemperatureFac::to_float(self.eps1_battery1_temperature)
    }

    pub fn eps1_battery2_v(&self) -> f32 {
        PayloadVoltageFac::to_float(self.eps1_battery2_v)
    }

    pub fn eps1_battery2_temperature(&self) -> f32 {
        PayloadTemperatureFac::to_float(self.eps1_battery2_temperature)
    }

    pub fn eps1_output_3v3_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps1_output_3v3_current)
    }

    pub fn eps1_output_3v3_overwrote(&self) -> bool {
        self.eps1_output_3v3_overwrote
    }

    pub fn eps1_output_3v3_status(&self) -> PowerOutputStatus {
        self.eps1_output_3v3_status
    }

    pub fn eps1_output_5v_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps1_output_5v_current)
    }

    pub fn eps1_output_5v_overwrote(&self) -> bool {
        self.eps1_output_5v_overwrote
    }

    pub fn eps1_output_5v_status(&self) -> PowerOutputStatus {
        self.eps1_output_5v_status
    }

    pub fn eps1_output_9v_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps1_output_9v_current)
    }

    pub fn eps1_output_9v_overwrote(&self) -> bool {
        self.eps1_output_9v_overwrote
    }

    pub fn eps1_output_9v_status(&self) -> PowerOutputStatus {
        self.eps1_output_9v_status
    }

    pub fn eps2_online(&self) -> bool {
        self.eps2_online
    }

    pub fn eps2_rebooted_in_last_5s(&self) -> bool {
        self.eps2_rebooted_in_last_5s
    }

    pub fn eps2_battery1_v(&self) -> f32 {
        PayloadVoltageFac::to_float(self.eps2_battery1_v)
    }

    pub fn eps2_battery1_temperature(&self) -> f32 {
        PayloadTemperatureFac::to_float(self.eps2_battery1_temperature)
    }

    pub fn eps2_battery2_v(&self) -> f32 {
        PayloadVoltageFac::to_float(self.eps2_battery2_v)
    }

    pub fn eps2_battery2_temperature(&self) -> f32 {
        PayloadTemperatureFac::to_float(self.eps2_battery2_temperature)
    }

    pub fn eps2_output_3v3_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps2_output_3v3_current)
    }

    pub fn eps2_output_3v3_overwrote(&self) -> bool {
        self.eps2_output_3v3_overwrote
    }

    pub fn eps2_output_3v3_status(&self) -> PowerOutputStatus {
        self.eps2_output_3v3_status
    }

    pub fn eps2_output_5v_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps2_output_5v_current)
    }

    pub fn eps2_output_5v_overwrote(&self) -> bool {
        self.eps2_output_5v_overwrote
    }

    pub fn eps2_output_5v_status(&self) -> PowerOutputStatus {
        self.eps2_output_5v_status
    }

    pub fn eps2_output_9v_current(&self) -> f32 {
        PayloadCurrentFac::to_float(self.eps2_output_9v_current)
    }

    pub fn eps2_output_9v_overwrote(&self) -> bool {
        self.eps2_output_9v_overwrote
    }

    pub fn eps2_output_9v_status(&self) -> PowerOutputStatus {
        self.eps2_output_9v_status
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            unix_clock_ready: self.unix_clock_ready(),
            num_of_fix_satellites: self.num_of_fix_satellites(),
            lat: self.lat(),
            lon: self.lon(),
            vl_battery_v: self.vl_battery_v(),
            air_temperature: self.air_temperature(),
            vl_stm32_temperature: self.vl_stm32_temperature(),
            pyro_main_continuity: self.pyro_main_continuity(),
            pyro_drogue_continuity: self.pyro_drogue_continuity(),
            altitude_agl: self.altitude_agl(),
            max_altitude_agl: self.max_altitude_agl(),
            backup_max_altitude_agl: self.backup_max_altitude_agl(),
            air_speed: self.air_speed(),
            max_air_speed: self.max_air_speed(),
            backup_max_air_speed: self.backup_max_air_speed(),
            tilt_deg: self.tilt_deg(),
            flight_stage: format!("{:?}", self.flight_stage()),
            backup_flight_stage: format!("{:?}", self.backup_flight_stage()),

            amp_online: self.amp_online(),
            amp_rebooted_in_last_5s: self.amp_rebooted_in_last_5s(),
            shared_battery_v: self.shared_battery_v(),
            amp_out1_overwrote: self.amp_out1_overwrote(),
            amp_out1: format!("{:?}", self.amp_out1()),
            amp_out2_overwrote: self.amp_out2_overwrote(),
            amp_out2: format!("{:?}", self.amp_out2()),
            amp_out3_overwrote: self.amp_out3_overwrote(),
            amp_out3: format!("{:?}", self.amp_out3()),
            amp_out4_overwrote: self.amp_out4_overwrote(),
            amp_out4: format!("{:?}", self.amp_out4()),

            main_bulkhead_online: self.main_bulkhead_online(),
            main_bulkhead_rebooted_in_last_5s: self.main_bulkhead_rebooted_in_last_5s(),
            main_bulkhead_brightness: self.main_bulkhead_brightness_lux(),

            drogue_bulkhead_online: self.drogue_bulkhead_online(),
            drogue_bulkhead_rebooted_in_last_5s: self.drogue_bulkhead_rebooted_in_last_5s(),
            drogue_bulkhead_brightness: self.drogue_bulkhead_brightness_lux(),

            icarus_online: self.icarus_online(),
            icarus_rebooted_in_last_5s: self.icarus_rebooted_in_last_5s(),

            air_brakes_commanded_extension_percentage: self.air_brakes_commanded_extension_percentage(),
            air_brakes_actual_extension_percentage: self.air_brakes_actual_extension_percentage(),
            air_brakes_servo_temp: self.air_brakes_servo_temp(),
            ap_residue: self.ap_residue(),
            cd: self.cd(),

            ozys1_online: self.ozys1_online(),
            ozys1_rebooted_in_last_5s: self.ozys1_rebooted_in_last_5s(),
            ozys2_online: self.ozys2_online(),
            ozys2_rebooted_in_last_5s: self.ozys2_rebooted_in_last_5s(),

            aero_rust_online: self.aero_rust_online(),
            aero_rust_rebooted_in_last_5s: self.aero_rust_rebooted_in_last_5s(),
            aero_rust_health: format!("{:?}", self.aero_rust_health()),

            payload_activation_pcb_online: self.payload_activation_pcb_online(),
            payload_activation_pcb_rebooted_in_last_5s: self.payload_activation_pcb_rebooted_in_last_5s(),

            rocket_wifi_online: self.rocket_wifi_online(),
            rocket_wifi_rebooted_in_last_5s: self.rocket_wifi_rebooted_in_last_5s(),

            eps1_online: self.eps1_online(),
            eps1_rebooted_in_last_5s: self.eps1_rebooted_in_last_5s(),
            eps1_battery1_v: self.eps1_battery1_v(),
            eps1_battery1_temperature: self.eps1_battery1_temperature(),
            eps1_battery2_v: self.eps1_battery2_v(),
            eps1_battery2_temperature: self.eps1_battery2_temperature(),
            eps1_output_3v3_current: self.eps1_output_3v3_current(),
            eps1_output_3v3_overwrote: self.eps1_output_3v3_overwrote(),
            eps1_output_3v3_status: format!("{:?}", self.eps1_output_3v3_status()),
            eps1_output_5v_current: self.eps1_output_5v_current(),
            eps1_output_5v_overwrote: self.eps1_output_5v_overwrote(),
            eps1_output_5v_status: format!("{:?}", self.eps1_output_5v_status()),
            eps1_output_9v_current: self.eps1_output_9v_current(),
            eps1_output_9v_overwrote: self.eps1_output_9v_overwrote(),
            eps1_output_9v_status: format!("{:?}", self.eps1_output_9v_status()),

            eps2_online: self.eps2_online(),
            eps2_rebooted_in_last_5s: self.eps2_rebooted_in_last_5s(),
            eps2_battery1_v: self.eps2_battery1_v(),
            eps2_battery1_temperature: self.eps2_battery1_temperature(),
            eps2_battery2_v: self.eps2_battery2_v(),
            eps2_battery2_temperature: self.eps2_battery2_temperature(),
            eps2_output_3v3_current: self.eps2_output_3v3_current(),
            eps2_output_3v3_overwrote: self.eps2_output_3v3_overwrote(),
            eps2_output_3v3_status: format!("{:?}", self.eps2_output_3v3_status()),
            eps2_output_5v_current: self.eps2_output_5v_current(),
            eps2_output_5v_overwrote: self.eps2_output_5v_overwrote(),
            eps2_output_5v_status: format!("{:?}", self.eps2_output_5v_status()),
            eps2_output_9v_current: self.eps2_output_9v_current(),
            eps2_output_9v_overwrote: self.eps2_output_9v_overwrote(),
            eps2_output_9v_status: format!("{:?}", self.eps2_output_9v_status()),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
        let nonce: u8 = self.nonce.into();
        let (lat, lon) = self.lat_lon();
        defmt::write!(f, "TelemetryPacket")
    }
}

impl Into<VLPDownlinkPacket> for TelemetryPacket {
    fn into(self) -> VLPDownlinkPacket {
        VLPDownlinkPacket::Telemetry(self)
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

    pub flight_stage: FlightStage,
    pub backup_flight_stage: FlightStage,

    pub amp_online: bool,
    pub amp_uptime_s: u32,
    pub shared_battery_v: f32,
    pub amp_out1_overwrote: bool,
    pub amp_out1: PowerOutputStatus,
    pub amp_out2_overwrote: bool,
    pub amp_out2: PowerOutputStatus,
    pub amp_out3_overwrote: bool,
    pub amp_out3: PowerOutputStatus,
    pub amp_out4_overwrote: bool,
    pub amp_out4: PowerOutputStatus,

    pub main_bulkhead_online: bool,
    pub main_bulkhead_uptime_s: u32,
    pub main_bulkhead_brightness: f32,

    pub drogue_bulkhead_online: bool,
    pub drogue_bulkhead_uptime_s: u32,
    pub drogue_bulkhead_brightness: f32,

    pub icarus_online: bool,
    pub icarus_uptime_s: u32,
    pub air_brakes_commanded_extension_percentage: f32,
    pub air_brakes_actual_extension_percentage: f32,
    pub air_brakes_servo_temp: f32,
    pub ap_residue: f32,
    pub cd: f32,

    pub ozys1_online: bool,
    pub ozys1_uptime_s: u32,

    pub ozys2_online: bool,
    pub ozys2_uptime_s: u32,

    pub aero_rust_uptime_s: u32,
    pub aero_rust_health: NodeHealth,
    pub aero_rust_mode: NodeMode,
    pub aero_rust_status: u16,

    pub payload_activation_pcb_online: bool,
    pub payload_activation_pcb_uptime_s: u32,

    pub rocket_wifi_online: bool,
    pub rocket_wifi_uptime_s: u32,

    pub eps1_online: bool,
    pub eps1_uptime_s: u32,
    pub eps1_battery1_v: f32,
    pub eps1_battery1_temperature: f32,
    pub eps1_battery2_v: f32,
    pub eps1_battery2_temperature: f32,
    pub eps1_output_3v3_current: f32,
    pub eps1_output_3v3_overwrote: bool,
    pub eps1_output_3v3_status: PowerOutputStatus,
    pub eps1_output_5v_current: f32,
    pub eps1_output_5v_overwrote: bool,
    pub eps1_output_5v_status: PowerOutputStatus,
    pub eps1_output_9v_current: f32,
    pub eps1_output_9v_overwrote: bool,
    pub eps1_output_9v_status: PowerOutputStatus,

    pub eps2_online: bool,
    pub eps2_uptime_s: u32,
    pub eps2_battery1_v: f32,
    pub eps2_battery1_temperature: f32,
    pub eps2_battery2_v: f32,
    pub eps2_battery2_temperature: f32,
    pub eps2_output_3v3_current: f32,
    pub eps2_output_3v3_overwrote: bool,
    pub eps2_output_3v3_status: PowerOutputStatus,
    pub eps2_output_5v_current: f32,
    pub eps2_output_5v_overwrote: bool,
    pub eps2_output_5v_status: PowerOutputStatus,
    pub eps2_output_9v_current: f32,
    pub eps2_output_9v_overwrote: bool,
    pub eps2_output_9v_status: PowerOutputStatus,
}

pub struct TelemetryPacketBuilder<M: RawMutex> {
    state: BlockingMutex<M, RefCell<TelemetryPacketBuilderState>>,
}

impl<M: RawMutex> TelemetryPacketBuilder<M> {
    pub fn new() -> Self {
        Self {
            state: BlockingMutex::new(RefCell::new(TelemetryPacketBuilderState {
                nonce: 0,

                gps_location: None,

                vl_battery_v: 0.0,
                air_temperature: 0.0,
                vl_stm32_temperature: 0.0,

                pyro_main_continuity: false,
                pyro_drogue_continuity: false,

                altitude: 0.0,
                max_altitude: 0.0,
                backup_altitude: 0.0,
                backup_max_altitude: 0.0,

                air_speed: 0.0,
                max_air_speed: 0.0,
                backup_air_speed: 0.0,
                backup_max_air_speed: 0.0,

                tilt_deg: 0.0,

                flight_stage: FlightStage::Armed,
                backup_flight_stage: FlightStage::Armed,

                amp_online: false,
                amp_uptime_s: 0,
                shared_battery_v: 0.0,
                amp_out1_overwrote: false,
                amp_out1: PowerOutputStatus::Disabled,
                amp_out2_overwrote: false,
                amp_out2: PowerOutputStatus::Disabled,
                amp_out3_overwrote: false,
                amp_out3: PowerOutputStatus::Disabled,
                amp_out4_overwrote: false,
                amp_out4: PowerOutputStatus::Disabled,

                main_bulkhead_online: false,
                main_bulkhead_uptime_s: 0,
                main_bulkhead_brightness: 0f32,

                drogue_bulkhead_online: false,
                drogue_bulkhead_uptime_s: 0,
                drogue_bulkhead_brightness: 0f32,

                icarus_online: false,
                icarus_uptime_s: 0,
                air_brakes_commanded_extension_percentage: 0.0,
                air_brakes_actual_extension_percentage: 0.0,
                air_brakes_servo_temp: 0.0,
                ap_residue: 0.0,
                cd: 0.0,

                ozys1_online: false,
                ozys1_uptime_s: 0,

                ozys2_online: false,
                ozys2_uptime_s: 0,

                aero_rust_uptime_s: 0,
                aero_rust_health: NodeHealth::Healthy,
                aero_rust_mode: NodeMode::Offline,
                aero_rust_status: 0,

                payload_activation_pcb_online: false,
                payload_activation_pcb_uptime_s: 0,

                rocket_wifi_online: false,
                rocket_wifi_uptime_s: 0,

                eps1_online: false,
                eps1_uptime_s: 0,
                eps1_battery1_v: 0.0,
                eps1_battery1_temperature: 0.0,
                eps1_battery2_v: 0.0,
                eps1_battery2_temperature: 0.0,
                eps1_output_3v3_current: 0.0,
                eps1_output_3v3_overwrote: false,
                eps1_output_3v3_status: PowerOutputStatus::Disabled,
                eps1_output_5v_current: 0.0,
                eps1_output_5v_overwrote: false,
                eps1_output_5v_status: PowerOutputStatus::Disabled,
                eps1_output_9v_current: 0.0,
                eps1_output_9v_overwrote: false,
                eps1_output_9v_status: PowerOutputStatus::Disabled,

                eps2_online: false,
                eps2_uptime_s: 0,
                eps2_battery1_v: 0.0,
                eps2_battery1_temperature: 0.0,
                eps2_battery2_v: 0.0,
                eps2_battery2_temperature: 0.0,
                eps2_output_3v3_current: 0.0,
                eps2_output_3v3_overwrote: false,
                eps2_output_3v3_status: PowerOutputStatus::Disabled,
                eps2_output_5v_current: 0.0,
                eps2_output_5v_overwrote: false,
                eps2_output_5v_status: PowerOutputStatus::Disabled,
                eps2_output_9v_current: 0.0,
                eps2_output_9v_overwrote: false,
                eps2_output_9v_status: PowerOutputStatus::Disabled,
            })),
        }
    }

    pub fn create_packet(&self) -> TelemetryPacket {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.nonce += 1;
            if state.nonce > 15 {
                state.nonce = 0;
            }
            TelemetryPacket::new(
                state.nonce,
                state
                    .gps_location
                    .as_ref()
                    .map(|g| g.timestamp)
                    .flatten()
                    .is_some(),
                state
                    .gps_location
                    .as_ref()
                    .map(|g| g.num_of_fix_satellites)
                    .unwrap_or(0),
                state.gps_location.as_ref().map(|g| g.lat_lon).flatten(),
                state.vl_battery_v,
                state.air_temperature,
                state.vl_stm32_temperature,
                state.pyro_main_continuity,
                state.pyro_drogue_continuity,
                state.altitude,
                state.max_altitude,
                state.backup_max_altitude,
                state.air_speed,
                state.max_air_speed,
                state.backup_max_air_speed,
                state.tilt_deg,
                state.flight_stage,
                state.backup_flight_stage,
                state.amp_online,
                state.amp_uptime_s < 5,
                state.shared_battery_v,
                state.amp_out1_overwrote,
                state.amp_out1,
                state.amp_out2_overwrote,
                state.amp_out2,
                state.amp_out3_overwrote,
                state.amp_out3,
                state.amp_out4_overwrote,
                state.amp_out4,
                state.main_bulkhead_online,
                state.main_bulkhead_uptime_s < 5,
                state.main_bulkhead_brightness,
                state.drogue_bulkhead_online,
                state.drogue_bulkhead_uptime_s < 5,
                state.drogue_bulkhead_brightness,
                state.icarus_online,
                state.icarus_uptime_s < 5,
                state.air_brakes_commanded_extension_percentage,
                state.air_brakes_actual_extension_percentage,
                state.air_brakes_servo_temp,
                state.ap_residue,
                state.cd,
                state.ozys1_online,
                state.ozys1_uptime_s < 5,
                state.ozys2_online,
                state.ozys2_uptime_s < 5,
                true,
                state.aero_rust_uptime_s < 5,
                state.aero_rust_health,
                state.payload_activation_pcb_online,
                state.payload_activation_pcb_uptime_s < 5,
                state.rocket_wifi_online,
                state.rocket_wifi_uptime_s < 5,
                state.eps1_online,
                state.eps1_uptime_s < 5,
                state.eps1_battery1_v,
                state.eps1_battery1_temperature,
                state.eps1_battery2_v,
                state.eps1_battery2_temperature,
                state.eps1_output_3v3_current,
                state.eps1_output_3v3_overwrote,
                state.eps1_output_3v3_status,
                state.eps1_output_5v_current,
                state.eps1_output_5v_overwrote,
                state.eps1_output_5v_status,
                state.eps1_output_9v_current,
                state.eps1_output_9v_overwrote,
                state.eps1_output_9v_status,
                state.eps2_online,
                state.eps2_uptime_s < 5,
                state.eps2_battery1_v,
                state.eps2_battery1_temperature,
                state.eps2_battery2_v,
                state.eps2_battery2_temperature,
                state.eps2_output_3v3_current,
                state.eps2_output_3v3_overwrote,
                state.eps2_output_3v3_status,
                state.eps2_output_5v_current,
                state.eps2_output_5v_overwrote,
                state.eps2_output_5v_status,
                state.eps2_output_9v_current,
                state.eps2_output_9v_overwrote,
                state.eps2_output_9v_status,
            )
        })
    }

    pub fn update<U>(&self, update_fn: U)
    where
        U: FnOnce(&mut RefMut<TelemetryPacketBuilderState>) -> (),
    {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            update_fn(&mut state);
            state.max_altitude = state.altitude.max(state.max_altitude);
            state.max_air_speed = state.air_speed.max(state.max_air_speed);
            state.backup_max_altitude = state.backup_altitude.max(state.backup_max_altitude);
            state.backup_max_air_speed = state.backup_air_speed.max(state.backup_max_air_speed);
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_encode_brightness_lux() {
        init_logger();

        let brightness_lux = 1000.0;
        let encoded = TelemetryPacket::encode_brightness_lux(brightness_lux);

        let decoded = TelemetryPacket::decode_brightness_lux(encoded);

        log_info!("original: {}", brightness_lux);
        log_info!("decoded: {}", decoded);
    }
}
