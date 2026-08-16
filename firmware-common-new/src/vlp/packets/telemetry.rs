use core::cell::{RefCell, RefMut};
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::RawMutex};
use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    can_bus::{
        custom_status::payload_sdrm_custom_status::PayloadSDRMCustomStatus,
        messages::{amp_status::PowerOutputStatus, vl_status::FlightStage},
    },
    fixed_point_factory,
    gps::GPSData,
};

use super::{TEMPERATURE_FAC_BITS, TemperatureFac, TemperatureFacBase, VLPDownlinkPacket};

/// Largest satellite count the 5-bit packet field can hold. Counts above this
/// are reported as this value; the SD log and the low-power / landed packets
/// carry the full `u8`.
pub const MAX_REPORTED_FIX_SATELLITES: u8 = 31;

// 23 bits for latitude, 24 bits for longitude
// resolution of 2.4m at equator
fixed_point_factory!(LatFac, f64, -90.0, 90.0, 0.00002146);
fixed_point_factory!(LonFac, f64, -180.0, 180.0, 0.00002146);

fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);
// 14 bits, ~0.62m resolution. Shared by every altitude in the packet, so they
// all clamp at the same ceiling rather than at four different ones.
fixed_point_factory!(AltitudeFac, f32, -100.0, 10000.0, 1.0);
// 10 bits, signed (negative = descending), ~1.42m/s resolution. Asymmetric on
// purpose: the ceiling covers Mach 3 (~1021m/s at sea level, less higher up)
// while the floor only has to cover a ballistic descent, so a symmetric range
// would spend a bit on speeds the rocket cannot reach going down.
fixed_point_factory!(VerticalVelocityFac, f32, -400.0, 1050.0, 2.0);
fixed_point_factory!(AirBrakesExtensionPercentFac, f32, 0.0, 1.0, 0.04);
fixed_point_factory!(TiltDegFac, f32, -90.0, 90.0, 1.0);

// EPM battery bus, a 4S-ish pack sitting well above the regulated rails. The
// floor is 0 rather than 11 V because an unavailable reading is sent as 0, and
// a floor of 11 would have decoded that as a plausible 11.0 V. Still 11 bits —
// the same width the old range plus its validity bit took.
fixed_point_factory!(EpmBattVFac, f32, 0.0, 17.0, 0.01);
// Load current of one EPM switched rail. 5A is the stack's design maximum, so
// the old 0..10.23A range was spending two bits per rail on current the
// hardware cannot draw; 7 bits over 0..5A is ~39mA per code. A rail somehow
// drawing more pins at 5.00A rather than wrapping. CAN and the SD slow record
// keep the full u16 mA, so an over-range fault is still exact in the log.
fixed_point_factory!(EpmRailMaFac, f32, 0.0, 5000.0, 40.0);
// SEM linear actuator position. The full u16 step range at ~64 step resolution;
// SEM's own step scale decides what that means in millimetres.
fixed_point_factory!(ActuatorStepsFac, f32, 0.0, 65535.0, 64.0);

// 293 bits = 36.625 bytes, so 37 with three spare bits. On air the packet
// costs `n + 1` bytes of data plus `(n + 1) / 4` of reed-solomon ecc, which
// puts this at 47 bytes on air. The symbol count steps at 50 / 55 / 60 bytes
// on air, so there is headroom to 38 bytes before the next step — but the
// struct is sized to its contents, not to the step, so growth is a deliberate
// edit rather than something that happens by accident.
// 250khz bandwidth + 12sf + 8cr lora, inside the 2s telemetry period.
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "37")]
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

    #[packed_field(element_size_bits = "14")]
    deployment_kf_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    #[packed_field(element_size_bits = "14")]
    max_deployment_kf_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    /// The deployment estimator's vertical velocity, signed (negative =
    /// descending). 0 during its Mach lockout, where the KF is frozen.
    #[packed_field(element_size_bits = "10")]
    deployment_kf_vertical_velocity:
        Integer<VerticalVelocityFacBase, packed_bits::Bits<VERTICAL_VELOCITY_FAC_BITS>>,

    #[packed_field(element_size_bits = "8")]
    airbrakes_kf_tilt_deg: Integer<TiltDegFacBase, packed_bits::Bits<TILT_DEG_FAC_BITS>>,

    /// All 8 codes are used, so a new `FlightStage` variant does not fit
    /// without widening this field.
    #[packed_field(element_size_bits = "3", ty = "enum")]
    flight_stage: FlightStage,

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    airbrakes_born: bool,
    /// The apogee AGL the MPC predicts at the extension it is commanding.
    /// Equal to `target_apogee_agl` while the target is reachable; the gap
    /// between them is the whole story of whether the brakes have authority.
    #[packed_field(element_size_bits = "14")]
    mpc_predicted_apogee_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    /// The configured target apogee, AGL.
    #[packed_field(element_size_bits = "14")]
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

    ozys_online: bool,
    ozys_rebooted_in_last_5s: bool,

    payload_sdrm_online: bool,
    payload_sdrm_rebooted_in_last_5s: bool,

    /// Payload stack flags, relayed from the SDRM node's
    /// `NodeStatusMessage::custom_status_raw`, one packet field per flag. The
    /// CAN field is 11 bits wide because that is what the heartbeat reserves
    /// for any node; only these 8 flags are defined, so only 8 are carried.
    payload_epm_alive: bool,
    payload_sem_alive: bool,
    payload_epm_rails_on: bool,
    payload_exp1_active: bool,
    payload_exp2_active: bool,
    payload_exp3_active: bool,
    payload_sdrm_sd_logging: bool,
    payload_sem_sd_logging: bool,

    /// Payload stack telemetry, relayed from `CustomPayloadStatusMessage`. A
    /// reading the payload could not take (`0xFFFF` on CAN) is sent as 0; the
    /// SD slow record keeps the `0xFFFF` sentinel, so that is where an
    /// unavailable reading stays distinguishable from a real zero.
    #[packed_field(element_size_bits = "11")]
    epm_batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,

    /// EPM switched rail load currents, 10mA resolution over 0..10.23A.
    #[packed_field(element_size_bits = "7")]
    epm_sys_3v3_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    epm_sys_5v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    epm_per_3v3_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    epm_per_5v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    epm_per_9v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    #[packed_field(element_size_bits = "7")]
    epm_per_12v_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,

    /// SEM linear actuator positions, ~64 step resolution over the full u16 range.
    #[packed_field(element_size_bits = "10")]
    sem_actuator_1_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_2_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
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

        deployment_kf_altitude_agl: f32,
        max_deployment_kf_altitude_agl: f32,
        deployment_kf_vertical_velocity: f32,

        airbrakes_kf_tilt_deg: f32,

        flight_stage: FlightStage,

        airbrakes_born: bool,
        mpc_predicted_apogee_agl: f32,
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

        ozys_online: bool,
        ozys_rebooted_in_last_5s: bool,

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
        if deployment_kf_altitude_agl.is_nan(){
            log_info!("altitude agl nan");
        }
        Self {
            nonce: nonce.into(),

            unix_clock_ready,
            // Saturate rather than let packed_struct truncate: 5 bits wrap at
            // 32, so a 32-satellite fix would downlink as 0 — exactly the
            // reading that means "no fix, do not fly". Clipping a very good
            // fix to "31" is harmless; inverting it is not.
            num_of_fix_satellites: num_of_fix_satellites
                .min(MAX_REPORTED_FIX_SATELLITES)
                .into(),
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),

            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),

            pyro_main_continuity,
            pyro_drogue_continuity,

            deployment_kf_altitude_agl: AltitudeFac::to_fixed_point_capped(deployment_kf_altitude_agl),
            max_deployment_kf_altitude_agl: AltitudeFac::to_fixed_point_capped(
                max_deployment_kf_altitude_agl,
            ),
            deployment_kf_vertical_velocity: VerticalVelocityFac::to_fixed_point_capped(
                deployment_kf_vertical_velocity,
            ),

            airbrakes_kf_tilt_deg: TiltDegFac::to_fixed_point_capped(airbrakes_kf_tilt_deg),

            flight_stage: flight_stage.into(),

            airbrakes_born,
            mpc_predicted_apogee_agl: AltitudeFac::to_fixed_point_capped(mpc_predicted_apogee_agl),
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

            ozys_online,
            ozys_rebooted_in_last_5s,

            payload_sdrm_online,
            payload_sdrm_rebooted_in_last_5s,

            payload_epm_alive: payload_stack_status.epm_alive,
            payload_sem_alive: payload_stack_status.sem_alive,
            payload_epm_rails_on: payload_stack_status.epm_rails_on,
            payload_exp1_active: payload_stack_status.exp1_active,
            payload_exp2_active: payload_stack_status.exp2_active,
            payload_exp3_active: payload_stack_status.exp3_active,
            payload_sdrm_sd_logging: payload_stack_status.sdrm_sd_logging,
            payload_sem_sd_logging: payload_stack_status.sem_sd_logging,

            // An unavailable reading is sent as 0 rather than carrying its own
            // validity bit.
            epm_batt_v: EpmBattVFac::to_fixed_point_capped(
                epm_batt_mv.map(|mv| mv as f32 / 1000.0).unwrap_or(0.0),
            ),

            epm_sys_3v3_ma: Self::encode_rail_ma(epm_rail_ma[0]),
            epm_sys_5v_ma: Self::encode_rail_ma(epm_rail_ma[1]),
            epm_per_3v3_ma: Self::encode_rail_ma(epm_rail_ma[2]),
            epm_per_5v_ma: Self::encode_rail_ma(epm_rail_ma[3]),
            epm_per_9v_ma: Self::encode_rail_ma(epm_rail_ma[4]),
            epm_per_12v_ma: Self::encode_rail_ma(epm_rail_ma[5]),

            sem_actuator_1_steps: Self::encode_steps(sem_actuator_steps[0]),
            sem_actuator_2_steps: Self::encode_steps(sem_actuator_steps[1]),
            sem_actuator_3_steps: Self::encode_steps(sem_actuator_steps[2]),
        }
    }

    fn encode_rail_ma(
        rail_ma: Option<u16>,
    ) -> Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>> {
        EpmRailMaFac::to_fixed_point_capped(rail_ma.unwrap_or(0) as f32)
    }

    fn decode_rail_ma(
        rail_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    ) -> u16 {
        libm::roundf(EpmRailMaFac::to_float(rail_ma)) as u16
    }

    fn encode_steps(
        steps: Option<u16>,
    ) -> Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>> {
        ActuatorStepsFac::to_fixed_point_capped(steps.unwrap_or(0) as f32)
    }

    fn decode_steps(
        steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    ) -> u16 {
        libm::roundf(ActuatorStepsFac::to_float(steps)) as u16
    }

    pub fn unix_clock_ready(&self) -> bool {
        self.unix_clock_ready
    }

    /// Saturating: [`MAX_REPORTED_FIX_SATELLITES`] means "at least that many".
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

    pub fn deployment_kf_altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.deployment_kf_altitude_agl)
    }

    pub fn max_deployment_kf_altitude_agl(&self) -> f32 {
        AltitudeFac::to_float(self.max_deployment_kf_altitude_agl)
    }

    pub fn airbrakes_kf_tilt_deg(&self) -> f32 {
        TiltDegFac::to_float(self.airbrakes_kf_tilt_deg)
    }

    pub fn flight_stage(&self) -> FlightStage {
        self.flight_stage
    }

    /// The deployment estimator's vertical velocity, signed (negative =
    /// descending). 0 during its Mach lockout, where the KF is frozen.
    pub fn deployment_kf_vertical_velocity(&self) -> f32 {
        VerticalVelocityFac::to_float(self.deployment_kf_vertical_velocity)
    }

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub fn airbrakes_born(&self) -> bool {
        self.airbrakes_born
    }

    /// The apogee AGL the MPC predicts at the extension it is commanding.
    pub fn mpc_predicted_apogee_agl(&self) -> f32 {
        AltitudeFac::to_float(self.mpc_predicted_apogee_agl)
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

    pub fn ozys_online(&self) -> bool {
        self.ozys_online
    }

    pub fn ozys_rebooted_in_last_5s(&self) -> bool {
        self.ozys_rebooted_in_last_5s
    }

    pub fn payload_sdrm_online(&self) -> bool {
        self.payload_sdrm_online
    }

    pub fn payload_sdrm_rebooted_in_last_5s(&self) -> bool {
        self.payload_sdrm_rebooted_in_last_5s
    }

    pub fn payload_epm_alive(&self) -> bool {
        self.payload_epm_alive
    }

    pub fn payload_sem_alive(&self) -> bool {
        self.payload_sem_alive
    }

    pub fn payload_epm_rails_on(&self) -> bool {
        self.payload_epm_rails_on
    }

    pub fn payload_exp1_active(&self) -> bool {
        self.payload_exp1_active
    }

    pub fn payload_exp2_active(&self) -> bool {
        self.payload_exp2_active
    }

    pub fn payload_exp3_active(&self) -> bool {
        self.payload_exp3_active
    }

    pub fn payload_sdrm_sd_logging(&self) -> bool {
        self.payload_sdrm_sd_logging
    }

    pub fn payload_sem_sd_logging(&self) -> bool {
        self.payload_sem_sd_logging
    }

    /// `None` when the payload reported the reading as unavailable.
    /// 0 when the payload could not read it. The SD slow record keeps the
    /// `0xFFFF` sentinel and is where that stays distinguishable.
    pub fn epm_batt_v(&self) -> f32 {
        EpmBattVFac::to_float(self.epm_batt_v)
    }

    pub fn epm_sys_3v3_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_sys_3v3_ma)
    }

    pub fn epm_sys_5v_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_sys_5v_ma)
    }

    pub fn epm_per_3v3_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_per_3v3_ma)
    }

    pub fn epm_per_5v_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_per_5v_ma)
    }

    pub fn epm_per_9v_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_per_9v_ma)
    }

    pub fn epm_per_12v_ma(&self) -> u16 {
        Self::decode_rail_ma(self.epm_per_12v_ma)
    }

    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub fn epm_rail_ma(&self) -> [u16; 6] {
        [
            self.epm_sys_3v3_ma(),
            self.epm_sys_5v_ma(),
            self.epm_per_3v3_ma(),
            self.epm_per_5v_ma(),
            self.epm_per_9v_ma(),
            self.epm_per_12v_ma(),
        ]
    }

    pub fn sem_actuator_1_steps(&self) -> u16 {
        Self::decode_steps(self.sem_actuator_1_steps)
    }

    pub fn sem_actuator_2_steps(&self) -> u16 {
        Self::decode_steps(self.sem_actuator_2_steps)
    }

    pub fn sem_actuator_3_steps(&self) -> u16 {
        Self::decode_steps(self.sem_actuator_3_steps)
    }

    /// Experiment channels 1..3.
    pub fn sem_actuator_steps(&self) -> [u16; 3] {
        [
            self.sem_actuator_1_steps(),
            self.sem_actuator_2_steps(),
            self.sem_actuator_3_steps(),
        ]
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
            pyro_main_continuity: self.pyro_main_continuity(),
            pyro_drogue_continuity: self.pyro_drogue_continuity(),
            deployment_kf_altitude_agl: self.deployment_kf_altitude_agl(),
            max_deployment_kf_altitude_agl: self.max_deployment_kf_altitude_agl(),
            deployment_kf_vertical_velocity: self.deployment_kf_vertical_velocity(),
            airbrakes_kf_tilt_deg: self.airbrakes_kf_tilt_deg(),
            flight_stage: format!("{:?}", self.flight_stage()),

            airbrakes_born: self.airbrakes_born(),
            mpc_predicted_apogee_agl: self.mpc_predicted_apogee_agl(),
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

            ozys_online: self.ozys_online(),
            ozys_rebooted_in_last_5s: self.ozys_rebooted_in_last_5s(),

            payload_sdrm_online: self.payload_sdrm_online(),
            payload_sdrm_rebooted_in_last_5s: self.payload_sdrm_rebooted_in_last_5s(),

            payload_epm_alive: self.payload_epm_alive(),
            payload_sem_alive: self.payload_sem_alive(),
            payload_epm_rails_on: self.payload_epm_rails_on(),
            payload_sdrm_sd_logging: self.payload_sdrm_sd_logging(),
            payload_sem_sd_logging: self.payload_sem_sd_logging(),
            payload_exp1_active: self.payload_exp1_active(),
            payload_exp2_active: self.payload_exp2_active(),
            payload_exp3_active: self.payload_exp3_active(),

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

    pub deployment_kf_altitude_agl: f32,
    max_deployment_kf_altitude_agl: f32,
    /// The deployment estimator's vertical velocity, signed. 0 during its
    /// Mach lockout, where the KF is frozen.
    pub deployment_kf_vertical_velocity: f32,

    pub airbrakes_kf_tilt_deg: f32,

    pub flight_stage: FlightStage,

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub airbrakes_born: bool,
    /// The apogee AGL the MPC predicts at the extension it is commanding.
    pub mpc_predicted_apogee_agl: f32,
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

    pub ozys_online: bool,
    pub ozys_uptime_s: u32,

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

                deployment_kf_altitude_agl: 0.0,
                max_deployment_kf_altitude_agl: 0.0,
                deployment_kf_vertical_velocity: 0.0,

                airbrakes_kf_tilt_deg: 0.0,

                flight_stage: FlightStage::Armed,

                airbrakes_born: false,
                mpc_predicted_apogee_agl: 0.0,
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

                ozys_online: false,
                ozys_uptime_s: 0,

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
                state.deployment_kf_altitude_agl,
                state.max_deployment_kf_altitude_agl,
                state.deployment_kf_vertical_velocity,
                state.airbrakes_kf_tilt_deg,
                state.flight_stage,
                state.airbrakes_born,
                state.mpc_predicted_apogee_agl,
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
                state.ozys_online,
                state.ozys_uptime_s < 5,
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
            state.max_deployment_kf_altitude_agl = state.deployment_kf_altitude_agl.max(state.max_deployment_kf_altitude_agl);
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
            -150.0,
            10.0,
            FlightStage::Ascent,
            true,
            2900.0,
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
            PayloadSDRMCustomStatus {
                epm_alive: true,
                sem_alive: false,
                epm_rails_on: true,
                exp1_active: false,
                exp2_active: true,
                exp3_active: false,
                sdrm_sd_logging: true,
                sem_sd_logging: false,
            },
            Some(12600),
            [Some(120), Some(340), None, Some(780), Some(1500), Some(2400)],
            [Some(0), Some(1200), Some(34567)],
        );
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);
        // 1 byte packet type + the 37 byte packed struct.
        assert_eq!(len, 38);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();
        assert_eq!(deserialized_packet, packet);

        let VLPDownlinkPacket::Telemetry(p) = deserialized_packet else {
            unreachable!()
        };
        // Deployment-estimator fields, at their widened ranges.
        assert_relative_eq!(p.deployment_kf_altitude_agl(), 1234.0, epsilon = 0.7);
        assert_eq!(p.flight_stage(), FlightStage::Ascent);

        assert_relative_eq!(p.deployment_kf_vertical_velocity(), -150.0, epsilon = 1.5);
        assert_relative_eq!(p.airbrakes_kf_tilt_deg(), 10.0, epsilon = 0.8);
        assert!(p.airbrakes_born());
        assert_relative_eq!(p.mpc_predicted_apogee_agl(), 2900.0, epsilon = 0.7);
        assert_relative_eq!(p.target_apogee_agl(), 3000.0, epsilon = 1.0);

        // Payload readings survive the round trip within their quantization
        // (~39mA per rail code, ~64 steps per actuator code). An unavailable
        // reading is sent as 0 -- there is no validity bit, so rail 2 here is
        // indistinguishable on the downlink from a rail drawing nothing.
        assert_relative_eq!(p.epm_batt_v(), 12.6, epsilon = 0.01);
        let rails = p.epm_rail_ma();
        assert_relative_eq!(rails[0] as f32, 120.0, epsilon = 40.0);
        assert_relative_eq!(rails[1] as f32, 340.0, epsilon = 40.0);
        assert_eq!(rails[2], 0);
        assert_relative_eq!(rails[3] as f32, 780.0, epsilon = 40.0);
        assert_relative_eq!(rails[4] as f32, 1500.0, epsilon = 40.0);
        assert_relative_eq!(rails[5] as f32, 2400.0, epsilon = 40.0);
        let steps = p.sem_actuator_steps();
        assert_eq!(steps[0], 0);
        assert_relative_eq!(steps[1] as f32, 1200.0, epsilon = 64.0);
        assert_relative_eq!(steps[2] as f32, 34567.0, epsilon = 64.0);

        // The stack flags are individual packet fields now, not a relayed
        // 11 bit blob.
        assert!(p.payload_epm_alive());
        assert!(!p.payload_sem_alive());
        assert!(p.payload_exp2_active());
    }

    fn packet_with_satellites(n: u8) -> TelemetryPacket {
        TelemetryPacket::new(
            10,
            true,
            n,
            Some((45.5, -73.6)),
            7.4,
            25.5,
            true,
            true,
            1234.0,
            2345.0,
            -150.0,
            10.0,
            FlightStage::Ascent,
            true,
            2900.0,
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
            PayloadSDRMCustomStatus::new(),
            Some(12600),
            [Some(120), Some(340), None, Some(780), Some(1500), Some(2400)],
            [Some(0), Some(1200), Some(34567)],
        )
    }

    /// The satellite count is 5 bits, so packed_struct truncates rather than
    /// clamps. Without the saturation in `new`, 32 satellites downlink as 0 --
    /// the reading that means "no fix, do not fly". Anything at or above the
    /// cap must read as the cap, never wrap.
    #[test]
    fn satellite_count_saturates_instead_of_wrapping() {
        init_logger();

        for n in 0..=MAX_REPORTED_FIX_SATELLITES {
            assert_eq!(packet_with_satellites(n).num_of_fix_satellites(), n);
        }
        for n in [
            MAX_REPORTED_FIX_SATELLITES + 1,
            MAX_REPORTED_FIX_SATELLITES + 2,
            40,
            99,
            u8::MAX,
        ] {
            assert_eq!(
                packet_with_satellites(n).num_of_fix_satellites(),
                MAX_REPORTED_FIX_SATELLITES,
                "{} satellites must saturate, not wrap",
                n
            );
        }
    }
}
