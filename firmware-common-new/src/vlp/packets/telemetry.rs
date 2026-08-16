use core::cell::{RefCell, RefMut};
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use micromath::F32Ext;
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    can_bus::{
        custom_status::{
            NodeCustomStatusExt, payload_sdrm_custom_status::PayloadSDRMCustomStatus,
        },
        messages::{amp_status::PowerOutputStatus, vl_status::FlightStage},
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
fixed_point_factory!(AltitudeFac, f32, -100.0, 7000.0, 1.0);
fixed_point_factory!(AirSpeedFac, f32, 0.0, 400.0, 2.0);
fixed_point_factory!(AirBrakesExtensionPercentFac, f32, 0.0, 1.0, 0.04);
fixed_point_factory!(TiltDegFac, f32, -90.0, 90.0, 1.0);

// EPM battery bus, a 4S-ish pack sitting well above the regulated rails.
fixed_point_factory!(EpmBattVFac, f32, 11.0, 17.0, 0.01);
// The four regulated EPM rails: 3.3V, system 5V, peripheral 5V and peripheral 9V.
fixed_point_factory!(EpmRailVFac, f32, 0.0, 10.0, 0.01);

// 48 byte max size to achieve 0.5Hz with 250khz bandwidth + 12sf + 8cr lora
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "34")]
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

    pyro_main_continuity: bool,
    pyro_drogue_continuity: bool,

    #[packed_field(element_size_bits = "13")]
    altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "13")]
    max_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,
    #[packed_field(element_size_bits = "8")]
    max_air_speed: Integer<AirSpeedFacBase, packed_bits::Bits<AIR_SPEED_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    tilt_deg: Integer<TiltDegFacBase, packed_bits::Bits<TILT_DEG_FAC_BITS>>,

    #[packed_field(element_size_bits = "4", ty = "enum")]
    flight_stage: FlightStage,
    /// Burn-timer flag from the deployment estimator — orthogonal to
    /// `flight_stage`, never folded into it.
    coasting: bool,
    /// `deployed` from `RocketState::DrogueChute` / `MainChute`.
    drogue_deployed: bool,
    main_deployed: bool,

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

    ozys1_online: bool,
    ozys1_rebooted_in_last_5s: bool,

    ozys2_online: bool,
    ozys2_rebooted_in_last_5s: bool,

    payload_sdrm_online: bool,
    payload_sdrm_rebooted_in_last_5s: bool,

    /// `PayloadSDRMCustomStatus`, relayed from
    /// `NodeStatusMessage::custom_status_raw` of the payload SDRM node.
    #[packed_field(element_size_bits = "11")]
    payload_stack_status_raw: Integer<u16, packed_bits::Bits<11>>,

    /// EPM rail voltages, relayed from `CustomPayloadStatusMessage`. Each rail
    /// carries a validity bit; the payload reports `0xFFFF` when a reading is
    /// unavailable, which arrives here as `false`.
    epm_batt_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,

    epm_sys_3v3_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_sys_3v3_v: Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>>,

    epm_sys_5v_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_sys_5v_v: Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>>,

    epm_per_5v_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_5v_v: Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>>,

    epm_per_9v_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_9v_v: Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>>,
}

impl TelemetryPacket {
    pub fn new(
        nonce: u8,

        unix_clock_ready: bool,
        num_of_fix_satellites: u8,
        lat_lon: Option<(f64, f64)>,

        vl_battery_v: f32,
        air_temperature: f32,

        pyro_main_continuity: bool,
        pyro_drogue_continuity: bool,

        altitude_agl: f32,
        max_altitude_agl: f32,

        air_speed: f32,
        max_air_speed: f32,

        tilt_deg: f32,

        flight_stage: FlightStage,
        coasting: bool,
        drogue_deployed: bool,
        main_deployed: bool,

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

        ozys1_online: bool,
        ozys1_rebooted_in_last_5s: bool,

        ozys2_online: bool,
        ozys2_rebooted_in_last_5s: bool,

        payload_sdrm_online: bool,
        payload_sdrm_rebooted_in_last_5s: bool,

        payload_stack_status: PayloadSDRMCustomStatus,

        epm_batt_v: Option<f32>,
        epm_sys_3v3_v: Option<f32>,
        epm_sys_5v_v: Option<f32>,
        epm_per_5v_v: Option<f32>,
        epm_per_9v_v: Option<f32>,
    ) -> Self {
        if altitude_agl.is_nan(){
            log_info!("altitude agl nan");
        }
        Self {
            nonce: nonce.into(),

            unix_clock_ready,
            num_of_fix_satellites: num_of_fix_satellites.into(),
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),

            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),

            pyro_main_continuity,
            pyro_drogue_continuity,

            altitude_agl: AltitudeFac::to_fixed_point_capped(altitude_agl),
            max_altitude_agl: AltitudeFac::to_fixed_point_capped(max_altitude_agl),

            air_speed: AirSpeedFac::to_fixed_point_capped(air_speed),
            max_air_speed: AirSpeedFac::to_fixed_point_capped(max_air_speed),

            tilt_deg: TiltDegFac::to_fixed_point_capped(tilt_deg),

            flight_stage: flight_stage.into(),
            coasting,
            drogue_deployed,
            main_deployed,

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

            ozys1_online,
            ozys1_rebooted_in_last_5s,

            ozys2_online,
            ozys2_rebooted_in_last_5s,

            payload_sdrm_online,
            payload_sdrm_rebooted_in_last_5s,

            payload_stack_status_raw: payload_stack_status.to_u16().into(),

            epm_batt_v_valid: epm_batt_v.is_some(),
            epm_batt_v: EpmBattVFac::to_fixed_point_capped(epm_batt_v.unwrap_or(11.0)),
            epm_sys_3v3_v_valid: epm_sys_3v3_v.is_some(),
            epm_sys_3v3_v: Self::encode_rail_v(epm_sys_3v3_v),
            epm_sys_5v_v_valid: epm_sys_5v_v.is_some(),
            epm_sys_5v_v: Self::encode_rail_v(epm_sys_5v_v),
            epm_per_5v_v_valid: epm_per_5v_v.is_some(),
            epm_per_5v_v: Self::encode_rail_v(epm_per_5v_v),
            epm_per_9v_v_valid: epm_per_9v_v.is_some(),
            epm_per_9v_v: Self::encode_rail_v(epm_per_9v_v),
        }
    }

    fn encode_rail_v(rail_v: Option<f32>) -> Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>> {
        EpmRailVFac::to_fixed_point_capped(rail_v.unwrap_or(0.0))
    }

    fn decode_rail_v(
        valid: bool,
        rail_v: Integer<EpmRailVFacBase, packed_bits::Bits<EPM_RAIL_V_FAC_BITS>>,
    ) -> Option<f32> {
        if valid {
            Some(EpmRailVFac::to_float(rail_v))
        } else {
            None
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

    pub fn air_speed(&self) -> f32 {
        AirSpeedFac::to_float(self.air_speed)
    }

    pub fn max_air_speed(&self) -> f32 {
        AirSpeedFac::to_float(self.max_air_speed)
    }

    pub fn tilt_deg(&self) -> f32 {
        TiltDegFac::to_float(self.tilt_deg)
    }

    pub fn flight_stage(&self) -> FlightStage {
        self.flight_stage
    }

    pub fn coasting(&self) -> bool {
        self.coasting
    }

    pub fn drogue_deployed(&self) -> bool {
        self.drogue_deployed
    }

    pub fn main_deployed(&self) -> bool {
        self.main_deployed
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

    pub fn payload_sdrm_online(&self) -> bool {
        self.payload_sdrm_online
    }

    pub fn payload_sdrm_rebooted_in_last_5s(&self) -> bool {
        self.payload_sdrm_rebooted_in_last_5s
    }

    pub fn payload_stack_status(&self) -> PayloadSDRMCustomStatus {
        PayloadSDRMCustomStatus::from_u16(self.payload_stack_status_raw.into())
    }

    /// `None` when the payload reported the rail as unavailable.
    pub fn epm_batt_v(&self) -> Option<f32> {
        if self.epm_batt_v_valid {
            Some(EpmBattVFac::to_float(self.epm_batt_v))
        } else {
            None
        }
    }

    pub fn epm_sys_3v3_v(&self) -> Option<f32> {
        Self::decode_rail_v(self.epm_sys_3v3_v_valid, self.epm_sys_3v3_v)
    }

    pub fn epm_sys_5v_v(&self) -> Option<f32> {
        Self::decode_rail_v(self.epm_sys_5v_v_valid, self.epm_sys_5v_v)
    }

    pub fn epm_per_5v_v(&self) -> Option<f32> {
        Self::decode_rail_v(self.epm_per_5v_v_valid, self.epm_per_5v_v)
    }

    pub fn epm_per_9v_v(&self) -> Option<f32> {
        Self::decode_rail_v(self.epm_per_9v_v_valid, self.epm_per_9v_v)
    }

    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        let payload_stack_status = self.payload_stack_status();
        json::object! {
            unix_clock_ready: self.unix_clock_ready(),
            num_of_fix_satellites: self.num_of_fix_satellites(),
            lat: self.lat(),
            lon: self.lon(),
            vl_battery_v: self.vl_battery_v(),
            air_temperature: self.air_temperature(),
            pyro_main_continuity: self.pyro_main_continuity(),
            pyro_drogue_continuity: self.pyro_drogue_continuity(),
            altitude_agl: self.altitude_agl(),
            max_altitude_agl: self.max_altitude_agl(),
            air_speed: self.air_speed(),
            max_air_speed: self.max_air_speed(),
            tilt_deg: self.tilt_deg(),
            flight_stage: format!("{:?}", self.flight_stage()),
            coasting: self.coasting(),
            drogue_deployed: self.drogue_deployed(),
            main_deployed: self.main_deployed(),

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

            ozys1_online: self.ozys1_online(),
            ozys1_rebooted_in_last_5s: self.ozys1_rebooted_in_last_5s(),
            ozys2_online: self.ozys2_online(),
            ozys2_rebooted_in_last_5s: self.ozys2_rebooted_in_last_5s(),

            payload_sdrm_online: self.payload_sdrm_online(),
            payload_sdrm_rebooted_in_last_5s: self.payload_sdrm_rebooted_in_last_5s(),

            payload_epm_alive: payload_stack_status.epm_alive,
            payload_sem_alive: payload_stack_status.sem_alive,
            payload_stack_powered: payload_stack_status.stack_powered,
            payload_sdrm_sd_logging: payload_stack_status.sdrm_sd_logging,
            payload_sem_sd_logging: payload_stack_status.sem_sd_logging,
            payload_exp1_active: payload_stack_status.exp1_active,
            payload_exp2_active: payload_stack_status.exp2_active,
            payload_exp3_active: payload_stack_status.exp3_active,
            payload_prep_complete: payload_stack_status.prep_complete,
            payload_armed_bundle_complete: payload_stack_status.armed_bundle_complete,
            payload_fault: payload_stack_status.fault,

            epm_batt_v: self.epm_batt_v(),
            epm_sys_3v3_v: self.epm_sys_3v3_v(),
            epm_sys_5v_v: self.epm_sys_5v_v(),
            epm_per_5v_v: self.epm_per_5v_v(),
            epm_per_9v_v: self.epm_per_9v_v(),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TelemetryPacket {
    fn format(&self, f: defmt::Formatter) {
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

    pub pyro_main_continuity: bool,
    pub pyro_drogue_continuity: bool,

    pub altitude_agl: f32,
    max_altitude_agl: f32,

    pub air_speed: f32,
    max_air_speed: f32,

    pub tilt_deg: f32,

    pub flight_stage: FlightStage,
    /// Burn-timer flag from the deployment estimator — orthogonal to
    /// `flight_stage`, never folded into it.
    pub coasting: bool,
    /// `deployed` from `RocketState::DrogueChute` / `MainChute`.
    pub drogue_deployed: bool,
    pub main_deployed: bool,

    pub amp_online: bool,
    pub amp_uptime_s: u32,
    pub shared_battery_v: f32,
    pub amp_out1_overwrote: bool,
    pub amp_out1: PowerOutputStatus,
    pub amp_out2_overwrote: bool,
    pub amp_out2: PowerOutputStatus,
    pub amp_out3_overwrote: bool,
    pub amp_out3: PowerOutputStatus,
    // pub amp_out4_overwrote: bool,
    // pub amp_out4: PowerOutputStatus,

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

    pub ozys1_online: bool,
    pub ozys1_uptime_s: u32,

    pub ozys2_online: bool,
    pub ozys2_uptime_s: u32,

    pub payload_sdrm_online: bool,
    pub payload_sdrm_uptime_s: u32,

    /// Stack flags from the payload SDRM node's `NodeStatusMessage`.
    pub payload_stack_status: PayloadSDRMCustomStatus,

    /// EPM rails from `CustomPayloadStatusMessage`. `None` while the payload has
    /// not reported, or when it reports a rail as unavailable.
    pub epm_batt_v: Option<f32>,
    pub epm_sys_3v3_v: Option<f32>,
    pub epm_sys_5v_v: Option<f32>,
    pub epm_per_5v_v: Option<f32>,
    pub epm_per_9v_v: Option<f32>,
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

                pyro_main_continuity: false,
                pyro_drogue_continuity: false,

                altitude_agl: 0.0,
                max_altitude_agl: 0.0,

                air_speed: 0.0,
                max_air_speed: 0.0,

                tilt_deg: 0.0,

                flight_stage: FlightStage::Armed,
                coasting: false,
                drogue_deployed: false,
                main_deployed: false,

                amp_online: false,
                amp_uptime_s: 0,
                shared_battery_v: 0.0,
                amp_out1_overwrote: false,
                amp_out1: PowerOutputStatus::Disabled,
                amp_out2_overwrote: false,
                amp_out2: PowerOutputStatus::Disabled,
                amp_out3_overwrote: false,
                amp_out3: PowerOutputStatus::Disabled,
                // amp_out4_overwrote: false,
                // amp_out4: PowerOutputStatus::Disabled,

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

                ozys1_online: false,
                ozys1_uptime_s: 0,

                ozys2_online: false,
                ozys2_uptime_s: 0,

                payload_sdrm_online: false,
                payload_sdrm_uptime_s: 0,

                payload_stack_status: PayloadSDRMCustomStatus::new(),

                epm_batt_v: None,
                epm_sys_3v3_v: None,
                epm_sys_5v_v: None,
                epm_per_5v_v: None,
                epm_per_9v_v: None,
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
                state.pyro_main_continuity,
                state.pyro_drogue_continuity,
                state.altitude_agl,
                state.max_altitude_agl,
                state.air_speed,
                state.max_air_speed,
                state.tilt_deg,
                state.flight_stage,
                state.coasting,
                state.drogue_deployed,
                state.main_deployed,
                state.amp_online,
                state.amp_uptime_s < 5,
                state.shared_battery_v,
                state.amp_out1_overwrote,
                state.amp_out1,
                state.amp_out2_overwrote,
                state.amp_out2,
                state.amp_out3_overwrote,
                state.amp_out3,
                // state.amp_out4_overwrote,
                // state.amp_out4,
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
                state.ozys1_online,
                state.ozys1_uptime_s < 5,
                state.ozys2_online,
                state.ozys2_uptime_s < 5,
                state.payload_sdrm_online,
                state.payload_sdrm_uptime_s < 5,
                state.payload_stack_status.clone(),
                state.epm_batt_v,
                state.epm_sys_3v3_v,
                state.epm_sys_5v_v,
                state.epm_per_5v_v,
                state.epm_per_9v_v,
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
            state.max_altitude_agl = state.altitude_agl.max(state.max_altitude_agl);
            state.max_air_speed = state.air_speed.max(state.max_air_speed);
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
