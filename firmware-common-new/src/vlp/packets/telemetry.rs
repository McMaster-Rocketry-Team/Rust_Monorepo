use core::cell::{RefCell, RefMut};
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
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
// 9 bits, signed: the airbrakes estimator's vertical velocity keeps its sign
// (unlike `AirSpeedFac`, which is an unsigned magnitude). ~1.6m/s resolution.
fixed_point_factory!(VerticalVelocityFac, f32, -400.0, 400.0, 2.0);
fixed_point_factory!(AirBrakesExtensionPercentFac, f32, 0.0, 1.0, 0.04);
fixed_point_factory!(TiltDegFac, f32, -90.0, 90.0, 1.0);

// EPM battery bus, a 4S-ish pack sitting well above the regulated rails.
fixed_point_factory!(EpmBattVFac, f32, 11.0, 17.0, 0.01);
// Load current of one EPM switched rail, 10mA resolution over 0..10.23A.
fixed_point_factory!(EpmRailMaFac, f32, 0.0, 10230.0, 10.0);
// SEM linear actuator position. The full u16 step range at ~64 step resolution;
// SEM's own step scale decides what that means in millimetres.
fixed_point_factory!(ActuatorStepsFac, f32, 0.0, 65535.0, 64.0);

// 341 bits = 42.625 bytes, so 43 bytes with three spare bits. With the 1 byte
// packet type and reed-solomon ecc (len/4) that is 55 bytes on air, 1774ms at
// 250khz bandwidth + 12sf + 8cr lora — still inside the 2s telemetry period.
// Trimming the struct back to 39 bytes (50 on air) would drop that to 1642ms,
// the same time-on-air as a 36 byte struct: the symbol count only steps at
// 50 / 55 / 60 bytes.
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "43")]
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
    /// `deployed` from `RocketState::DrogueChute` / `MainChute`.
    drogue_deployed: bool,
    main_deployed: bool,

    /// The airbrakes estimator's altitude relative to the pad.
    #[packed_field(element_size_bits = "13")]
    ab_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    /// The airbrakes estimator's vertical velocity, signed (negative = descending).
    #[packed_field(element_size_bits = "9")]
    ab_vertical_velocity:
        Integer<VerticalVelocityFacBase, packed_bits::Bits<VERTICAL_VELOCITY_FAC_BITS>>,
    /// The mach-lockout-exit drag vote: the drag-inverted airspeed is
    /// currently below Mach 0.8. Sustained for 1 s, this is what opens the
    /// lockout — it is the whole exit criterion, not one vote of three.
    ab_vote_drag: bool,
    /// The airbrakes estimator's vertical filter is born (baro trusted).
    ab_born: bool,
    ab_apogee: bool,
    /// The configured target apogee, AGL.
    #[packed_field(element_size_bits = "13")]
    target_apogee_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

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

    /// Payload stack telemetry, relayed from `CustomPayloadStatusMessage`. Every
    /// reading carries a validity bit; the payload reports `0xFFFF` when a value
    /// is unavailable, which arrives here as `false`.
    epm_batt_v_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,

    /// EPM switched rail load currents, 10mA resolution over 0..10.23A.
    epm_sys_3v3_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_sys_3v3_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    epm_sys_5v_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_sys_5v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    epm_per_3v3_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_3v3_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    epm_per_5v_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_5v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    epm_per_9v_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_9v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    epm_per_12v_ma_valid: bool,
    #[packed_field(element_size_bits = "10")]
    epm_per_12v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    /// SEM linear actuator positions, ~64 step resolution over the full u16 range.
    sem_actuator_1_steps_valid: bool,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_1_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,

    sem_actuator_2_steps_valid: bool,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_2_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,

    sem_actuator_3_steps_valid: bool,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_3_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
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
        drogue_deployed: bool,
        main_deployed: bool,

        ab_altitude_agl: f32,
        ab_vertical_velocity: f32,
        ab_vote_drag: bool,
        ab_born: bool,
        ab_apogee: bool,
        target_apogee_agl: f32,

        amp_online: bool,
        amp_rebooted_in_last_5s: bool,
        shared_battery_v: f32,
        amp_out1_overwrote: bool,
        amp_out1: PowerOutputStatus,
        amp_out2_overwrote: bool,
        amp_out2: PowerOutputStatus,
        amp_out3_overwrote: bool,
        amp_out3: PowerOutputStatus,

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

        epm_batt_mv: Option<u16>,
        // Rail index order: 0 SYS_3V3, 1 SYS_5V, 2 PER_3V3, 3 PER_5V, 4 PER_9V,
        // 5 PER_12V.
        epm_rail_ma: [Option<u16>; 6],
        // Experiment channels 1..3.
        sem_actuator_steps: [Option<u16>; 3],
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
            drogue_deployed,
            main_deployed,

            ab_altitude_agl: AltitudeFac::to_fixed_point_capped(ab_altitude_agl),
            ab_vertical_velocity: VerticalVelocityFac::to_fixed_point_capped(ab_vertical_velocity),
            ab_vote_drag,
            ab_born,
            ab_apogee,
            target_apogee_agl: AltitudeFac::to_fixed_point_capped(target_apogee_agl),

            amp_online,
            amp_rebooted_in_last_5s,
            shared_battery_v: BatteryVFac::to_fixed_point_capped(shared_battery_v),

            amp_out1_overwrote,
            amp_out1,
            amp_out2_overwrote,
            amp_out2,
            amp_out3_overwrote,
            amp_out3,

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

            epm_batt_v_valid: epm_batt_mv.is_some(),
            epm_batt_v: EpmBattVFac::to_fixed_point_capped(
                epm_batt_mv.map(|mv| mv as f32 / 1000.0).unwrap_or(11.0),
            ),

            epm_sys_3v3_ma_valid: epm_rail_ma[0].is_some(),
            epm_sys_3v3_ma: Self::encode_rail_ma(epm_rail_ma[0]),
            epm_sys_5v_ma_valid: epm_rail_ma[1].is_some(),
            epm_sys_5v_ma: Self::encode_rail_ma(epm_rail_ma[1]),
            epm_per_3v3_ma_valid: epm_rail_ma[2].is_some(),
            epm_per_3v3_ma: Self::encode_rail_ma(epm_rail_ma[2]),
            epm_per_5v_ma_valid: epm_rail_ma[3].is_some(),
            epm_per_5v_ma: Self::encode_rail_ma(epm_rail_ma[3]),
            epm_per_9v_ma_valid: epm_rail_ma[4].is_some(),
            epm_per_9v_ma: Self::encode_rail_ma(epm_rail_ma[4]),
            epm_per_12v_ma_valid: epm_rail_ma[5].is_some(),
            epm_per_12v_ma: Self::encode_rail_ma(epm_rail_ma[5]),

            sem_actuator_1_steps_valid: sem_actuator_steps[0].is_some(),
            sem_actuator_1_steps: Self::encode_steps(sem_actuator_steps[0]),
            sem_actuator_2_steps_valid: sem_actuator_steps[1].is_some(),
            sem_actuator_2_steps: Self::encode_steps(sem_actuator_steps[1]),
            sem_actuator_3_steps_valid: sem_actuator_steps[2].is_some(),
            sem_actuator_3_steps: Self::encode_steps(sem_actuator_steps[2]),
        }
    }

    fn encode_rail_ma(
        rail_ma: Option<u16>,
    ) -> Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>> {
        EpmRailMaFac::to_fixed_point_capped(rail_ma.unwrap_or(0) as f32)
    }

    fn decode_rail_ma(
        valid: bool,
        rail_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    ) -> Option<u16> {
        if valid {
            Some(libm::roundf(EpmRailMaFac::to_float(rail_ma)) as u16)
        } else {
            None
        }
    }

    fn encode_steps(
        steps: Option<u16>,
    ) -> Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>> {
        ActuatorStepsFac::to_fixed_point_capped(steps.unwrap_or(0) as f32)
    }

    fn decode_steps(
        valid: bool,
        steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    ) -> Option<u16> {
        if valid {
            Some(libm::roundf(ActuatorStepsFac::to_float(steps)) as u16)
        } else {
            None
        }
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

    pub fn drogue_deployed(&self) -> bool {
        self.drogue_deployed
    }

    pub fn main_deployed(&self) -> bool {
        self.main_deployed
    }

    /// The airbrakes estimator's altitude relative to the pad.
    pub fn ab_altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.ab_altitude_agl)
    }

    /// The airbrakes estimator's vertical velocity, signed (negative = descending).
    pub fn ab_vertical_velocity(&self) -> f32 {
        VerticalVelocityFac::to_float(self.ab_vertical_velocity)
    }

    /// The mach-lockout-exit drag vote (see the field docs).
    pub fn ab_vote_drag(&self) -> bool {
        self.ab_vote_drag
    }

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub fn ab_born(&self) -> bool {
        self.ab_born
    }

    pub fn ab_apogee(&self) -> bool {
        self.ab_apogee
    }

    /// The configured target apogee, AGL.
    pub fn target_apogee_agl(&self) -> f32 {
        AltitudeFac::to_float(self.target_apogee_agl)
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

    /// `None` when the payload reported the reading as unavailable.
    pub fn epm_batt_v(&self) -> Option<f32> {
        if self.epm_batt_v_valid {
            Some(EpmBattVFac::to_float(self.epm_batt_v))
        } else {
            None
        }
    }

    pub fn epm_sys_3v3_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_sys_3v3_ma_valid, self.epm_sys_3v3_ma)
    }

    pub fn epm_sys_5v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_sys_5v_ma_valid, self.epm_sys_5v_ma)
    }

    pub fn epm_per_3v3_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_3v3_ma_valid, self.epm_per_3v3_ma)
    }

    pub fn epm_per_5v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_5v_ma_valid, self.epm_per_5v_ma)
    }

    pub fn epm_per_9v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_9v_ma_valid, self.epm_per_9v_ma)
    }

    pub fn epm_per_12v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_12v_ma_valid, self.epm_per_12v_ma)
    }

    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub fn epm_rail_ma(&self) -> [Option<u16>; 6] {
        [
            self.epm_sys_3v3_ma(),
            self.epm_sys_5v_ma(),
            self.epm_per_3v3_ma(),
            self.epm_per_5v_ma(),
            self.epm_per_9v_ma(),
            self.epm_per_12v_ma(),
        ]
    }

    pub fn sem_actuator_1_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_1_steps_valid, self.sem_actuator_1_steps)
    }

    pub fn sem_actuator_2_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_2_steps_valid, self.sem_actuator_2_steps)
    }

    pub fn sem_actuator_3_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_3_steps_valid, self.sem_actuator_3_steps)
    }

    /// Experiment channels 1..3.
    pub fn sem_actuator_steps(&self) -> [Option<u16>; 3] {
        [
            self.sem_actuator_1_steps(),
            self.sem_actuator_2_steps(),
            self.sem_actuator_3_steps(),
        ]
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
            drogue_deployed: self.drogue_deployed(),
            main_deployed: self.main_deployed(),

            ab_altitude_agl: self.ab_altitude_agl(),
            ab_vertical_velocity: self.ab_vertical_velocity(),
            ab_vote_drag: self.ab_vote_drag(),
            ab_born: self.ab_born(),
            ab_apogee: self.ab_apogee(),
            target_apogee_agl: self.target_apogee_agl(),

            amp_online: self.amp_online(),
            amp_rebooted_in_last_5s: self.amp_rebooted_in_last_5s(),
            shared_battery_v: self.shared_battery_v(),
            amp_out1_overwrote: self.amp_out1_overwrote(),
            amp_out1: format!("{:?}", self.amp_out1()),
            amp_out2_overwrote: self.amp_out2_overwrote(),
            amp_out2: format!("{:?}", self.amp_out2()),
            amp_out3_overwrote: self.amp_out3_overwrote(),
            amp_out3: format!("{:?}", self.amp_out3()),

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
            payload_epm_rails_on: payload_stack_status.epm_rails_on,
            payload_sdrm_sd_logging: payload_stack_status.sdrm_sd_logging,
            payload_sem_sd_logging: payload_stack_status.sem_sd_logging,
            payload_exp1_active: payload_stack_status.exp1_active,
            payload_exp2_active: payload_stack_status.exp2_active,
            payload_exp3_active: payload_stack_status.exp3_active,

            epm_batt_v: self.epm_batt_v(),
            epm_sys_3v3_ma: self.epm_sys_3v3_ma(),
            epm_sys_5v_ma: self.epm_sys_5v_ma(),
            epm_per_3v3_ma: self.epm_per_3v3_ma(),
            epm_per_5v_ma: self.epm_per_5v_ma(),
            epm_per_9v_ma: self.epm_per_9v_ma(),
            epm_per_12v_ma: self.epm_per_12v_ma(),

            sem_actuator_1_steps: self.sem_actuator_1_steps(),
            sem_actuator_2_steps: self.sem_actuator_2_steps(),
            sem_actuator_3_steps: self.sem_actuator_3_steps(),
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
    /// `deployed` from `RocketState::DrogueChute` / `MainChute`.
    pub drogue_deployed: bool,
    pub main_deployed: bool,

    /// The airbrakes estimator's altitude relative to the pad.
    pub ab_altitude_agl: f32,
    /// The airbrakes estimator's vertical velocity, signed (negative = descending).
    pub ab_vertical_velocity: f32,
    /// The mach-lockout-exit drag vote: the drag-inverted airspeed is
    /// below Mach 0.8. Sustained 1 s, this is what opens the lockout.
    pub ab_vote_drag: bool,
    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub ab_born: bool,
    pub ab_apogee: bool,
    /// The configured target apogee, AGL.
    pub target_apogee_agl: f32,

    pub amp_online: bool,
    pub amp_uptime_s: u32,
    pub shared_battery_v: f32,
    pub amp_out1_overwrote: bool,
    pub amp_out1: PowerOutputStatus,
    pub amp_out2_overwrote: bool,
    pub amp_out2: PowerOutputStatus,
    pub amp_out3_overwrote: bool,
    pub amp_out3: PowerOutputStatus,

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

    /// Payload stack telemetry from `CustomPayloadStatusMessage`, in the units it
    /// arrives in. `None` while the payload has not reported, or when it reports
    /// a reading as unavailable.
    pub epm_batt_mv: Option<u16>,
    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub epm_rail_ma: [Option<u16>; 6],
    /// Experiment channels 1..3.
    pub sem_actuator_steps: [Option<u16>; 3],
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
                drogue_deployed: false,
                main_deployed: false,

                ab_altitude_agl: 0.0,
                ab_vertical_velocity: 0.0,
                ab_vote_drag: false,
                ab_born: false,
                ab_apogee: false,
                target_apogee_agl: 0.0,

                amp_online: false,
                amp_uptime_s: 0,
                shared_battery_v: 0.0,
                amp_out1_overwrote: false,
                amp_out1: PowerOutputStatus::Disabled,
                amp_out2_overwrote: false,
                amp_out2: PowerOutputStatus::Disabled,
                amp_out3_overwrote: false,
                amp_out3: PowerOutputStatus::Disabled,

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

                epm_batt_mv: None,
                epm_rail_ma: [None; 6],
                sem_actuator_steps: [None; 3],
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
                state.drogue_deployed,
                state.main_deployed,
                state.ab_altitude_agl,
                state.ab_vertical_velocity,
                state.ab_vote_drag,
                state.ab_born,
                state.ab_apogee,
                state.target_apogee_agl,
                state.amp_online,
                state.amp_uptime_s < 5,
                state.shared_battery_v,
                state.amp_out1_overwrote,
                state.amp_out1,
                state.amp_out2_overwrote,
                state.amp_out2,
                state.amp_out3_overwrote,
                state.amp_out3,
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
                state.epm_batt_mv,
                state.epm_rail_ma,
                state.sem_actuator_steps,
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
    use approx::assert_relative_eq;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        init_logger();

        let packet = TelemetryPacket::new(
            10,
            true,
            12,
            Some((45.5, -73.6)),
            7.4,
            25.5,
            true,
            true,
            1234.0,
            2345.0,
            250.0,
            300.0,
            10.0,
            FlightStage::Ascent,
            false,
            false,
            1230.0,
            -150.0,
            true,
            true,
            false,
            3000.0,
            true,
            false,
            8.2,
            false,
            PowerOutputStatus::PowerGood,
            false,
            PowerOutputStatus::PowerGood,
            true,
            PowerOutputStatus::Disabled,
            true,
            false,
            0.5,
            0.45,
            42.0,
            true,
            false,
            true,
            false,
            true,
            false,
            PayloadSDRMCustomStatus::new(),
            Some(12600),
            [Some(120), Some(340), None, Some(780), Some(1500), Some(2400)],
            [Some(0), Some(1200), Some(34567)],
        );
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);
        // 1 byte packet type + the 43 byte packed struct.
        assert_eq!(len, 44);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();
        assert_eq!(deserialized_packet, packet);

        let VLPDownlinkPacket::Telemetry(p) = deserialized_packet else {
            unreachable!()
        };
        assert_relative_eq!(p.ab_altitude_agl(), 1230.0, epsilon = 1.0);
        assert_relative_eq!(p.ab_vertical_velocity(), -150.0, epsilon = 2.0);
        assert!(p.ab_vote_drag());
        assert!(p.ab_born());
        assert!(!p.ab_apogee());
        assert_relative_eq!(p.target_apogee_agl(), 3000.0, epsilon = 1.0);

        // Payload readings survive the round trip within their quantization
        // (10mA per rail code, ~64 steps per actuator code), and an unavailable
        // reading stays unavailable rather than decoding as 0.
        assert_relative_eq!(p.epm_batt_v().unwrap(), 12.6, epsilon = 0.01);
        let rails = p.epm_rail_ma();
        assert_relative_eq!(rails[0].unwrap() as f32, 120.0, epsilon = 10.0);
        assert_relative_eq!(rails[1].unwrap() as f32, 340.0, epsilon = 10.0);
        assert_eq!(rails[2], None);
        assert_relative_eq!(rails[3].unwrap() as f32, 780.0, epsilon = 10.0);
        assert_relative_eq!(rails[4].unwrap() as f32, 1500.0, epsilon = 10.0);
        assert_relative_eq!(rails[5].unwrap() as f32, 2400.0, epsilon = 10.0);
        let steps = p.sem_actuator_steps();
        assert_eq!(steps[0], Some(0));
        assert_relative_eq!(steps[1].unwrap() as f32, 1200.0, epsilon = 64.0);
        assert_relative_eq!(steps[2].unwrap() as f32, 34567.0, epsilon = 64.0);
    }
}
