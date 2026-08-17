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

// EPM battery bus, a 4S-ish pack sitting well above the regulated rails.
// 11 bits over 0..17V, so (17 - 0) / (2^11 - 1) = 8.3mV per code. The floor is
// 0 rather than 11 V because a collapsed / disconnected battery bus reading
// 0.0 V is a real fault the ground should see; a floor of 11 would have
// decoded that as a plausible 11.0 V. Absence is the all-ones code (see
// `EPM_BATT_V_UNAVAILABLE_CODE`), not 0, precisely so that 0.0 V stays
// available for that fault. Real readings therefore cap one code below full
// scale, at 16.992 V.
fixed_point_factory!(EpmBattVFac, f32, 0.0, 17.0, 0.01);
// Load current of one EPM switched rail. 5A is the stack's design maximum, so
// the old 0..10.23A range was spending two bits per rail on current the
// hardware cannot draw; 7 bits over 0..5A is (5000 - 0) / (2^7 - 1) = 39.4mA
// per code. CAN and the SD slow record keep the full u16 mA, so an over-range
// fault is still exact in the log. The all-ones code is reserved for absence,
// so a rail is reported saturated at 4.961A rather than 5.000A — the top code
// buys the ability to tell "rail switched off, drawing 0mA" (a normal state,
// see `payload_epm_rails_on`) apart from "EPM never reported this rail".
fixed_point_factory!(EpmRailMaFac, f32, 0.0, 5000.0, 40.0);
// SEM linear actuator position. The full u16 step range at
// (65535 - 0) / (2^10 - 1) = 64.1 steps per code; SEM's own step scale decides
// what that means in millimetres. As with the rails the all-ones code is
// absence, so real positions cap at 65471 steps — an actuator parked at step 0
// (the home position, which is where they sit for most of a flight) has to
// stay distinguishable from an actuator SEM never reported.
fixed_point_factory!(ActuatorStepsFac, f32, 0.0, 65535.0, 64.0);

// The all-ones code of each payload-relayed field, spent on "the payload could
// not take this reading" instead of on a value. The top of the range is the
// cheapest code to give up: the bottom of every one of these three ranges is a
// reading the ground genuinely needs to be able to see — 0.0 V is a collapsed
// battery bus, 0mA is a switched rail that is off, 0 steps is an actuator at
// home — whereas the top is saturation, which is already an approximation.
// `encode_*` therefore clamps real values one code below full scale, so a
// present reading can never collide with the sentinel.
const EPM_BATT_V_UNAVAILABLE_CODE: EpmBattVFacBase = (1 << EPM_BATT_V_FAC_BITS) - 1;
const EPM_RAIL_MA_UNAVAILABLE_CODE: EpmRailMaFacBase = (1 << EPM_RAIL_MA_FAC_BITS) - 1;
const ACTUATOR_STEPS_UNAVAILABLE_CODE: ActuatorStepsFacBase = (1 << ACTUATOR_STEPS_FAC_BITS) - 1;

/// Collapses NaN into `None`. A NaN reading is an absent reading that lost its
/// `Option` somewhere upstream, and every `fixed_point_factory` panics on one
/// (see the comment in [`TelemetryPacket::new`]), so it is folded back into
/// absence at the packet boundary.
fn defined(value: Option<f32>) -> Option<f32> {
    value.filter(|v| !v.is_nan())
}

// 299 bits = 37.375 bytes, so 38 with five spare bits. On air the packet costs
// `n + 1` bytes of data plus `(n + 1) / 4` of reed-solomon ecc, which puts
// this at 48 bytes on air. The symbol count steps at 50 / 55 / 60 bytes on
// air, so 38 bytes is the last size that still fits in the current symbol
// count — there is no room left to grow without paying for more air time
// inside the 2s telemetry period. The struct is sized to its contents, not to
// the step, so growth is a deliberate edit rather than something that happens
// by accident.
//
// Six of those bits are validity bits. The packet is a bit-packed
// `packed_struct` and so cannot carry an `Option` on the wire; the rule the
// rest of the codebase follows — absence is an `Option`, sentinels only where
// the medium cannot carry one — is honoured at the boundary instead. Every
// field with an absence encoding is set from an `Option` in `new` and read
// back as an `Option` by its getter, so no caller can confuse "the estimator
// had nothing to say" with a real zero.
// 250khz bandwidth + 12sf + 8cr lora, inside the 2s telemetry period.
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "38")]
pub struct TelemetryPacket {
    #[packed_field(bits = "0..4")]
    nonce: Integer<u8, packed_bits::Bits<4>>,

    unix_clock_ready: bool,
    num_of_fix_satellites: Integer<u8, packed_bits::Bits<5>>,
    /// Whether `lat` / `lon` hold a position at all. Absent until the GPS
    /// parses a fix, and absent again whenever it loses one.
    ///
    /// This is not the same question as `num_of_fix_satellites > 0`: the two
    /// come from independent NMEA fields, and a receiver that is tracking
    /// satellites but has not yet solved a position reports a nonzero count
    /// with no latitude or longitude. Reading the position off the satellite
    /// count would put the rocket at Null Island for that window, which is
    /// exactly the kind of plausible-looking wrong answer a recovery team
    /// would drive towards.
    lat_lon_valid: bool,
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

    /// Whether the deployment estimator produced an altitude and a vertical
    /// velocity for this packet. One bit for both because they are born and
    /// retired together — they are two components of the same Kalman state,
    /// and there is no situation in which the estimator has one but not the
    /// other.
    ///
    /// Absent in exactly two cases, matching
    /// `RocketStateEstimator::kf_altitude_asl`: before the filter is born (it
    /// has had no sample to run on yet), and throughout the Mach lockout,
    /// where it is frozen and holds a pre-ignition reading that goes stale by
    /// tens of seconds and kilometres.
    ///
    /// Present in every other state, INCLUDING on the pad, after touchdown and
    /// in `FailedToReachMinApogee`. Those `RocketState` variants carry no
    /// altitude field of their own, but the filter is running and fusing baro
    /// in all of them — the variants omit the number because the state machine
    /// has nothing to decide from it there, not because it is untrustworthy.
    /// The pad case is worth the bit on its own: a real near-zero AGL before
    /// launch is what tells the ground the baro and the filter are alive,
    /// which the hard 0.0 this replaced could never distinguish itself from.
    ///
    /// The bit exists because the Mach lockout window used to downlink a hard
    /// 0.0 while the SD log kept the frozen stale value, so a ground display
    /// and a post-flight plot disagreed about the same seconds of the flight.
    /// `flight_stage` cannot substitute for it: the lockout is folded into
    /// `Ascent` on the wire, so the stage does not change when the numbers
    /// stop being real.
    deployment_kf_valid: bool,
    #[packed_field(element_size_bits = "14")]
    deployment_kf_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,

    /// Whether `max_deployment_kf_altitude_agl` has ever been fed a real
    /// sample. Its own bit rather than a share of `deployment_kf_valid`
    /// because the two have different lifetimes: the running maximum is
    /// latched, so once the estimator has produced a single altitude it stays
    /// meaningful for the rest of the flight, including through the Mach
    /// lockout and after touchdown. Folding it into `deployment_kf_valid`
    /// would blank the apogee readout at exactly the moment the ground crew
    /// wants it — the `Landed` packet. It is absent only before the first
    /// sample, where the alternative is reporting a max altitude of 0m that
    /// looks like a measurement.
    max_deployment_kf_altitude_valid: bool,
    #[packed_field(element_size_bits = "14")]
    max_deployment_kf_altitude_agl: Integer<AltitudeFacBase, packed_bits::Bits<ALTITUDE_FAC_BITS>>,
    /// The deployment estimator's vertical velocity, signed (negative =
    /// descending). Guarded by `deployment_kf_valid`.
    #[packed_field(element_size_bits = "10")]
    deployment_kf_vertical_velocity:
        Integer<VerticalVelocityFacBase, packed_bits::Bits<VERTICAL_VELOCITY_FAC_BITS>>,

    /// Whether the airbrakes estimator produced a tilt for this packet. Tilt
    /// is gyro dead reckoning that only exists while the airbrakes estimator
    /// does: absent before ignition, and absent again once the estimator is
    /// retired at apogee. Zero degrees is a perfectly ordinary reading for a
    /// rocket going straight up, which is why absence needs its own bit here
    /// rather than a magic value.
    airbrakes_kf_tilt_valid: bool,
    #[packed_field(element_size_bits = "8")]
    airbrakes_kf_tilt_deg: Integer<TiltDegFacBase, packed_bits::Bits<TILT_DEG_FAC_BITS>>,

    /// All 8 codes are used, so a new `FlightStage` variant does not fit
    /// without widening this field.
    #[packed_field(element_size_bits = "3", ty = "enum")]
    flight_stage: FlightStage,

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    airbrakes_born: bool,
    /// The airbrakes estimator's pad calibration is complete.
    ///
    /// The one bit in this packet that is actionable BEFORE launch. The
    /// estimator refuses to detect ignition without a calibration, so a
    /// rocket sitting on the rail with this clear will fly with no airbrakes
    /// and nothing else in the downlink will say so — `airbrakes_born` is
    /// sourced from `baro_trusted`, which cannot go true until after the Mach
    /// lockout. Treat it as a go/no-go item.
    ///
    /// It is re-derived every 2 s from the last minute of pad data and can go
    /// back to false, so it reports the pad as it is NOW rather than latching
    /// the first success.
    airbrakes_calibrated: bool,
    /// Whether the MPC produced an apogee prediction for this packet. Absent
    /// whenever the controller is not solving: before the airbrakes estimator
    /// is born, after the controller is shut down at apogee, and on any cycle
    /// where the solver returns NaN. That last case used to be coerced to 0.0
    /// with a comment admitting the packet had no way to say "no prediction",
    /// which downlinked a 0m predicted apogee — the reading that otherwise
    /// means "the rocket will not leave the pad".
    mpc_predicted_apogee_valid: bool,
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
    /// Whether Icarus has reported the two fields below at least once. They
    /// arrive together in a single `IcarusStatusMessage`, so one bit covers
    /// both.
    ///
    /// `icarus_online` is not a substitute. That flag tracks the CAN
    /// heartbeat, which Icarus starts sending as soon as it boots — before it
    /// has sent any `IcarusStatusMessage`. In that window `icarus_online` is
    /// true while the two fields still hold their initial 0.0, i.e. "brakes
    /// fully retracted, servo at 0C", which reads as a measurement. The
    /// converse case is covered either way: once Icarus drops off the bus the
    /// last values go stale, and `icarus_online` going false is what says so.
    icarus_status_valid: bool,
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
    /// reading the payload could not take (`0xFFFF` on CAN) is sent as the
    /// all-ones code of the field, which the getters decode back to `None`.
    /// These three groups keep a sentinel instead of taking validity bits
    /// because ten readings would cost ten bits and the packet has five left;
    /// spending the top code of each field costs nothing but one quantum of
    /// headroom at full scale, which for a battery bus, a rail current and an
    /// actuator position is headroom nothing real ever reaches.
    #[packed_field(element_size_bits = "11")]
    epm_batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,

    /// EPM switched rail load currents, 39.4mA resolution over 0..4.961A.
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

    /// SEM linear actuator positions, 64.1 step resolution over 0..65471 steps.
    #[packed_field(element_size_bits = "10")]
    sem_actuator_1_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_2_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    #[packed_field(element_size_bits = "10")]
    sem_actuator_3_steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
}

/// The deployment estimator's live state, the pair of numbers the deployment
/// logic actually acts on. Grouped because they share one validity bit on the
/// wire: passing them as one `Option` is what makes it impossible to downlink
/// a real altitude next to an absent velocity, or vice versa.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeploymentKfState {
    /// Altitude above the launch pad, metres.
    pub altitude_agl: f32,
    /// Signed, negative = descending. Metres per second.
    pub vertical_velocity: f32,
}

/// What Icarus reports the air brakes are actually doing, as opposed to what
/// they were commanded to do. Grouped for the same reason as
/// [`DeploymentKfState`]: both numbers arrive in one `IcarusStatusMessage` and
/// share one validity bit, so they are present or absent together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IcarusAirBrakesState {
    /// Measured extension, 0..1.
    pub actual_extension_percentage: f32,
    /// Servo temperature, C.
    pub servo_temp: f32,
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

        deployment_kf: Option<DeploymentKfState>,
        max_deployment_kf_altitude_agl: Option<f32>,

        airbrakes_kf_tilt_deg: Option<f32>,

        flight_stage: FlightStage,

        airbrakes_born: bool,
        airbrakes_calibrated: bool,
        mpc_predicted_apogee_agl: Option<f32>,
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
        icarus_air_brakes: Option<IcarusAirBrakesState>,

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
        // A NaN that reaches a `fixed_point_factory` panics: the min/max
        // comparisons in `to_fixed_point_capped` are both false for NaN, so it
        // falls through to a float-to-int cast that has no answer and
        // `unwrap`s a `None`. NaN out of an estimator or a solver is absence
        // wearing a number's clothes, so fold it into the validity bit here
        // rather than letting it take the radio task down mid-flight.
        let deployment_kf = deployment_kf.filter(|s| {
            !s.altitude_agl.is_nan() && !s.vertical_velocity.is_nan()
        });
        let max_deployment_kf_altitude_agl = defined(max_deployment_kf_altitude_agl);
        let airbrakes_kf_tilt_deg = defined(airbrakes_kf_tilt_deg);
        let mpc_predicted_apogee_agl = defined(mpc_predicted_apogee_agl);
        let icarus_air_brakes = icarus_air_brakes.filter(|s| {
            !s.actual_extension_percentage.is_nan() && !s.servo_temp.is_nan()
        });

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
            lat_lon_valid: lat_lon.is_some(),
            // The 0.0 filler is never read back: `lat_lon` refuses to decode
            // unless `lat_lon_valid` is set.
            lat: LatFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).0),
            lon: LonFac::to_fixed_point_capped(lat_lon.unwrap_or((0.0, 0.0)).1),

            vl_battery_v: BatteryVFac::to_fixed_point_capped(vl_battery_v),
            air_temperature: TemperatureFac::to_fixed_point_capped(air_temperature),

            pyro_main_continuity,
            pyro_drogue_continuity,

            deployment_kf_valid: deployment_kf.is_some(),
            deployment_kf_altitude_agl: AltitudeFac::to_fixed_point_capped(
                deployment_kf.map(|s| s.altitude_agl).unwrap_or(0.0),
            ),

            max_deployment_kf_altitude_valid: max_deployment_kf_altitude_agl.is_some(),
            max_deployment_kf_altitude_agl: AltitudeFac::to_fixed_point_capped(
                max_deployment_kf_altitude_agl.unwrap_or(0.0),
            ),
            deployment_kf_vertical_velocity: VerticalVelocityFac::to_fixed_point_capped(
                deployment_kf.map(|s| s.vertical_velocity).unwrap_or(0.0),
            ),

            airbrakes_kf_tilt_valid: airbrakes_kf_tilt_deg.is_some(),
            airbrakes_kf_tilt_deg: TiltDegFac::to_fixed_point_capped(
                airbrakes_kf_tilt_deg.unwrap_or(0.0),
            ),

            flight_stage: flight_stage.into(),

            airbrakes_born,
            airbrakes_calibrated,
            mpc_predicted_apogee_valid: mpc_predicted_apogee_agl.is_some(),
            mpc_predicted_apogee_agl: AltitudeFac::to_fixed_point_capped(
                mpc_predicted_apogee_agl.unwrap_or(0.0),
            ),
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
            icarus_status_valid: icarus_air_brakes.is_some(),
            air_brakes_actual_extension_percentage:
                AirBrakesExtensionPercentFac::to_fixed_point_capped(
                    icarus_air_brakes
                        .map(|s| s.actual_extension_percentage)
                        .unwrap_or(0.0),
                ),
            air_brakes_servo_temp: TemperatureFac::to_fixed_point_capped(
                icarus_air_brakes.map(|s| s.servo_temp).unwrap_or(0.0),
            ),

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

            // An unavailable reading is sent as the field's all-ones code
            // rather than carrying its own validity bit.
            epm_batt_v: Self::encode_batt_v(epm_batt_mv),

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

    fn encode_batt_v(
        batt_mv: Option<u16>,
    ) -> Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>> {
        match batt_mv {
            None => EPM_BATT_V_UNAVAILABLE_CODE.into(),
            Some(mv) => {
                let code: EpmBattVFacBase =
                    EpmBattVFac::to_fixed_point_capped(mv as f32 / 1000.0).into();
                code.min(EPM_BATT_V_UNAVAILABLE_CODE - 1).into()
            }
        }
    }

    fn decode_batt_v(
        batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,
    ) -> Option<f32> {
        let code: EpmBattVFacBase = batt_v.into();
        if code == EPM_BATT_V_UNAVAILABLE_CODE {
            None
        } else {
            Some(EpmBattVFac::to_float(batt_v))
        }
    }

    fn encode_rail_ma(
        rail_ma: Option<u16>,
    ) -> Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>> {
        match rail_ma {
            None => EPM_RAIL_MA_UNAVAILABLE_CODE.into(),
            Some(ma) => {
                let code: EpmRailMaFacBase =
                    EpmRailMaFac::to_fixed_point_capped(ma as f32).into();
                code.min(EPM_RAIL_MA_UNAVAILABLE_CODE - 1).into()
            }
        }
    }

    fn decode_rail_ma(
        rail_ma: Integer<EpmRailMaFacBase, packed_bits::Bits<EPM_RAIL_MA_FAC_BITS>>,
    ) -> Option<u16> {
        let code: EpmRailMaFacBase = rail_ma.into();
        if code == EPM_RAIL_MA_UNAVAILABLE_CODE {
            None
        } else {
            Some(libm::roundf(EpmRailMaFac::to_float(rail_ma)) as u16)
        }
    }

    fn encode_steps(
        steps: Option<u16>,
    ) -> Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>> {
        match steps {
            None => ACTUATOR_STEPS_UNAVAILABLE_CODE.into(),
            Some(steps) => {
                let code: ActuatorStepsFacBase =
                    ActuatorStepsFac::to_fixed_point_capped(steps as f32).into();
                code.min(ACTUATOR_STEPS_UNAVAILABLE_CODE - 1).into()
            }
        }
    }

    fn decode_steps(
        steps: Integer<ActuatorStepsFacBase, packed_bits::Bits<ACTUATOR_STEPS_FAC_BITS>>,
    ) -> Option<u16> {
        let code: ActuatorStepsFacBase = steps.into();
        if code == ACTUATOR_STEPS_UNAVAILABLE_CODE {
            None
        } else {
            Some(libm::roundf(ActuatorStepsFac::to_float(steps)) as u16)
        }
    }

    pub fn unix_clock_ready(&self) -> bool {
        self.unix_clock_ready
    }

    /// Saturating: [`MAX_REPORTED_FIX_SATELLITES`] means "at least that many".
    pub fn num_of_fix_satellites(&self) -> u8 {
        self.num_of_fix_satellites.into()
    }

    /// `None` until the GPS has solved a position, and `None` again whenever
    /// it loses the fix.
    ///
    /// One getter rather than a `lat()` and a `lon()` because a latitude
    /// without its longitude is not a usable answer, and two getters would
    /// have meant two chances to skip the fix check. There is deliberately no
    /// way to read the raw fields: with no fix they hold 0.0, and a recovery
    /// team following a bearing to (0, 0) is the failure this getter exists to
    /// prevent.
    pub fn lat_lon(&self) -> Option<(f64, f64)> {
        if self.lat_lon_valid {
            Some((LatFac::to_float(self.lat), LonFac::to_float(self.lon)))
        } else {
            None
        }
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

    /// Altitude and signed vertical velocity together, as the deployment
    /// estimator produced them. `None` before the filter is born and through
    /// the Mach lockout, and `Some` everywhere else including the pad — see
    /// `deployment_kf_valid`.
    pub fn deployment_kf(&self) -> Option<DeploymentKfState> {
        if self.deployment_kf_valid {
            Some(DeploymentKfState {
                altitude_agl: AltitudeFac::to_float(self.deployment_kf_altitude_agl),
                vertical_velocity: VerticalVelocityFac::to_float(
                    self.deployment_kf_vertical_velocity,
                ),
            })
        } else {
            None
        }
    }

    /// `None` whenever the deployment estimator had no altitude for this
    /// packet. Notably `None` for the whole Mach lockout, where the SD log
    /// holds the frozen last-good value instead — the two are meant to
    /// disagree there, and this is how the downlink says so.
    pub fn deployment_kf_altitude_agl(&self) -> Option<f32> {
        self.deployment_kf().map(|s| s.altitude_agl)
    }

    /// The highest altitude the deployment estimator has reported so far.
    /// `None` only before it has produced its first sample; unlike
    /// [`Self::deployment_kf_altitude_agl`] this stays `Some` through the Mach
    /// lockout and after landing, because a latched maximum does not stop
    /// being true when the filter stops running.
    pub fn max_deployment_kf_altitude_agl(&self) -> Option<f32> {
        if self.max_deployment_kf_altitude_valid {
            Some(AltitudeFac::to_float(self.max_deployment_kf_altitude_agl))
        } else {
            None
        }
    }

    /// Tilt from vertical, from the airbrakes estimator's gyro dead
    /// reckoning. `None` before ignition and after the estimator is retired at
    /// apogee.
    pub fn airbrakes_kf_tilt_deg(&self) -> Option<f32> {
        if self.airbrakes_kf_tilt_valid {
            Some(TiltDegFac::to_float(self.airbrakes_kf_tilt_deg))
        } else {
            None
        }
    }

    pub fn flight_stage(&self) -> FlightStage {
        self.flight_stage
    }

    /// The deployment estimator's vertical velocity, signed (negative =
    /// descending). `None` wherever [`Self::deployment_kf_altitude_agl`] is —
    /// the two share one validity bit.
    pub fn deployment_kf_vertical_velocity(&self) -> Option<f32> {
        self.deployment_kf().map(|s| s.vertical_velocity)
    }

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub fn airbrakes_born(&self) -> bool {
        self.airbrakes_born
    }

    /// The airbrakes estimator's pad calibration is complete — the only
    /// pre-launch go/no-go bit in this packet. False on the rail means the
    /// airbrakes will not fly, silently.
    pub fn airbrakes_calibrated(&self) -> bool {
        self.airbrakes_calibrated
    }

    /// The apogee AGL the MPC predicts at the extension it is commanding.
    /// `None` whenever the controller is not solving — before the airbrakes
    /// estimator is born, after it is shut down at apogee, and on a cycle
    /// where the solver produced NaN.
    pub fn mpc_predicted_apogee_agl(&self) -> Option<f32> {
        if self.mpc_predicted_apogee_valid {
            Some(AltitudeFac::to_float(self.mpc_predicted_apogee_agl))
        } else {
            None
        }
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

    /// What Icarus reports the brakes are actually doing. `None` until Icarus
    /// has sent its first `IcarusStatusMessage` — which is later than
    /// [`Self::icarus_online`] going true, because the heartbeat starts at
    /// boot and the status message does not.
    pub fn icarus_air_brakes(&self) -> Option<IcarusAirBrakesState> {
        if self.icarus_status_valid {
            Some(IcarusAirBrakesState {
                actual_extension_percentage: AirBrakesExtensionPercentFac::to_float(
                    self.air_brakes_actual_extension_percentage,
                ),
                servo_temp: TemperatureFac::to_float(self.air_brakes_servo_temp),
            })
        } else {
            None
        }
    }

    pub fn air_brakes_actual_extension_percentage(&self) -> Option<f32> {
        self.icarus_air_brakes()
            .map(|s| s.actual_extension_percentage)
    }

    pub fn air_brakes_servo_temp(&self) -> Option<f32> {
        self.icarus_air_brakes().map(|s| s.servo_temp)
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

    /// EPM battery bus voltage. `None` when the payload reported the reading
    /// as unavailable (`0xFFFF` on CAN) or has not reported at all. A real
    /// 0.0 V — a collapsed or disconnected bus — decodes as `Some(0.0)`, not
    /// as absence; that distinction is the reason the sentinel is the top code
    /// rather than the bottom one.
    pub fn epm_batt_v(&self) -> Option<f32> {
        Self::decode_batt_v(self.epm_batt_v)
    }

    /// `None` when EPM could not read the rail. A rail that is switched off
    /// reads `Some(0)`, which is the normal state whenever
    /// [`Self::payload_epm_rails_on`] is false.
    pub fn epm_sys_3v3_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_sys_3v3_ma)
    }

    pub fn epm_sys_5v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_sys_5v_ma)
    }

    pub fn epm_per_3v3_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_3v3_ma)
    }

    pub fn epm_per_5v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_5v_ma)
    }

    pub fn epm_per_9v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_9v_ma)
    }

    pub fn epm_per_12v_ma(&self) -> Option<u16> {
        Self::decode_rail_ma(self.epm_per_12v_ma)
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

    /// `None` when SEM could not read the actuator. An actuator parked at its
    /// home position reads `Some(0)`.
    pub fn sem_actuator_1_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_1_steps)
    }

    pub fn sem_actuator_2_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_2_steps)
    }

    pub fn sem_actuator_3_steps(&self) -> Option<u16> {
        Self::decode_steps(self.sem_actuator_3_steps)
    }

    /// Experiment channels 1..3.
    pub fn sem_actuator_steps(&self) -> [Option<u16>; 3] {
        [
            self.sem_actuator_1_steps(),
            self.sem_actuator_2_steps(),
            self.sem_actuator_3_steps(),
        ]
    }

    /// Every field with an absence encoding serialises as JSON `null` when it
    /// is absent, so a consumer never has to know which sentinel or validity
    /// bit stands behind it. The key set is unchanged — `lat` and `lon` are
    /// still two keys even though they are read through one getter — so only
    /// the value type moved, from a fake number to `null`.
    #[cfg(feature = "json")]
    pub fn to_json(&self) -> json::JsonValue {
        json::object! {
            unix_clock_ready: self.unix_clock_ready(),
            num_of_fix_satellites: self.num_of_fix_satellites(),
            lat: self.lat_lon().map(|(lat, _)| lat),
            lon: self.lat_lon().map(|(_, lon)| lon),
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
            airbrakes_calibrated: self.airbrakes_calibrated(),
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

    /// The deployment estimator's altitude and signed vertical velocity.
    /// `None` whenever the estimator has nothing to say — before the filter is
    /// born, and through the Mach lockout where the KF is frozen. Feed it
    /// straight from `RocketStateEstimator::kf_altitude_asl` /
    /// `kf_vertical_velocity` rather than deriving absence from
    /// `RocketState`: the SD log reads those same two accessors, so sourcing
    /// both channels from one place is what keeps them from describing the
    /// same second of flight differently. Pass `None` rather than zeros —
    /// zeros are a reading, and the downlink now says which it is.
    pub deployment_kf: Option<DeploymentKfState>,
    /// Running maximum of `deployment_kf.altitude_agl`, maintained by
    /// [`TelemetryPacketBuilder::update`]. Latched: once the estimator has
    /// produced one sample this stays `Some` for the rest of the flight, so
    /// the apogee is still readable in the `Landed` packet.
    max_deployment_kf_altitude_agl: Option<f32>,

    /// Tilt from the airbrakes estimator's gyro dead reckoning. `None` before
    /// ignition and after the estimator is retired at apogee.
    pub airbrakes_kf_tilt_deg: Option<f32>,

    pub flight_stage: FlightStage,

    /// The airbrakes estimator's vertical filter is born (baro trusted).
    pub airbrakes_born: bool,
    /// The airbrakes estimator's pad calibration is complete. The one
    /// pre-launch go/no-go bit here: false on the rail means the airbrakes
    /// will not fly.
    pub airbrakes_calibrated: bool,
    /// The apogee AGL the MPC predicts at the extension it is commanding.
    /// `None` whenever the controller is not solving, including a cycle whose
    /// solution came back NaN.
    pub mpc_predicted_apogee_agl: Option<f32>,
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
    /// What Icarus reports the brakes are doing, from `IcarusStatusMessage`.
    /// `None` until the first one arrives, which is strictly after
    /// `icarus_online` goes true.
    pub icarus_air_brakes: Option<IcarusAirBrakesState>,

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

                deployment_kf: None,
                max_deployment_kf_altitude_agl: None,

                airbrakes_kf_tilt_deg: None,

                flight_stage: FlightStage::Armed,

                airbrakes_born: false,
                airbrakes_calibrated: false,
                mpc_predicted_apogee_agl: None,
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
                icarus_air_brakes: None,

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
                state.deployment_kf,
                state.max_deployment_kf_altitude_agl,
                state.airbrakes_kf_tilt_deg,
                state.flight_stage,
                state.airbrakes_born,
                state.airbrakes_calibrated,
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
                state.icarus_air_brakes,
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
            // Only a present altitude can move the maximum, and once it has
            // moved it never goes back to absent: an absent estimator says
            // nothing about the apogee already reached. This is what lets the
            // `Landed` packet still carry the max while the instantaneous
            // altitude is `None`.
            if let Some(altitude_agl) = state.deployment_kf.map(|s| s.altitude_agl) {
                state.max_deployment_kf_altitude_agl = Some(
                    state
                        .max_deployment_kf_altitude_agl
                        .map_or(altitude_agl, |max| max.max(altitude_agl)),
                );
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::tests::init_logger;

    use super::*;

    /// Every optional field present, so a round trip exercises the value side
    /// of each validity bit and each sentinel.
    fn packet_with_everything(num_of_fix_satellites: u8) -> TelemetryPacket {
        TelemetryPacket::new(
            10,
            true,
            num_of_fix_satellites,
            Some((45.5, -73.6)),
            7.4,
            25.5,
            true,
            true,
            Some(DeploymentKfState {
                altitude_agl: 1234.0,
                vertical_velocity: -150.0,
            }),
            Some(2345.0),
            Some(10.0),
            FlightStage::Ascent,
            true,
            true,
            Some(2900.0),
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
            Some(IcarusAirBrakesState {
                actual_extension_percentage: 0.45,
                servo_temp: 42.0,
            }),
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
        )
    }

    /// The pad / Mach-lockout / post-landing shape: nothing the estimators,
    /// the MPC, Icarus, the GPS or the payload produce is available yet.
    fn packet_with_nothing() -> TelemetryPacket {
        TelemetryPacket::new(
            10,
            false,
            0,
            None,
            7.4,
            25.5,
            true,
            true,
            None,
            None,
            None,
            FlightStage::Armed,
            false,
            false,
            None,
            3000.0,
            false,
            false,
            8.2,
            false,
            PowerOutputStatus::Disabled,
            false,
            PowerOutputStatus::Disabled,
            false,
            PowerOutputStatus::Disabled,
            false,
            false,
            0.0,
            None,
            false,
            false,
            false,
            false,
            PayloadSDRMCustomStatus::new(),
            None,
            [None; 6],
            [None; 3],
        )
    }

    fn round_trip(packet: TelemetryPacket) -> TelemetryPacket {
        let packet: VLPDownlinkPacket = packet.into();

        let mut buffer = [0u8; 64];
        let len = packet.serialize(&mut buffer);
        // 1 byte packet type + the 38 byte packed struct.
        assert_eq!(len, 39);

        let deserialized_packet = VLPDownlinkPacket::deserialize(&buffer[..len]).unwrap();
        assert_eq!(deserialized_packet, packet);

        let VLPDownlinkPacket::Telemetry(p) = deserialized_packet else {
            unreachable!()
        };
        p
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();

        let p = round_trip(packet_with_everything(12));

        let (lat, lon) = p.lat_lon().unwrap();
        assert_relative_eq!(lat, 45.5, epsilon = 0.0001);
        assert_relative_eq!(lon, -73.6, epsilon = 0.0001);

        // Deployment-estimator fields, at their widened ranges.
        assert_relative_eq!(
            p.deployment_kf_altitude_agl().unwrap(),
            1234.0,
            epsilon = 0.7
        );
        assert_eq!(p.flight_stage(), FlightStage::Ascent);

        assert_relative_eq!(
            p.deployment_kf_vertical_velocity().unwrap(),
            -150.0,
            epsilon = 1.5
        );
        assert_relative_eq!(
            p.max_deployment_kf_altitude_agl().unwrap(),
            2345.0,
            epsilon = 0.7
        );
        assert_relative_eq!(p.airbrakes_kf_tilt_deg().unwrap(), 10.0, epsilon = 0.8);
        assert!(p.airbrakes_born());
        assert_relative_eq!(
            p.mpc_predicted_apogee_agl().unwrap(),
            2900.0,
            epsilon = 0.7
        );
        assert_relative_eq!(p.target_apogee_agl(), 3000.0, epsilon = 1.0);

        assert_relative_eq!(
            p.air_brakes_actual_extension_percentage().unwrap(),
            0.45,
            epsilon = 0.02
        );
        assert_relative_eq!(p.air_brakes_servo_temp().unwrap(), 42.0, epsilon = 0.2);

        // Payload readings survive the round trip within their quantization
        // (39.4mA per rail code, 64.1 steps per actuator code). Rail 2 was
        // reported unavailable and comes back as `None`, not as a rail
        // drawing nothing -- and actuator 1, which really is at step 0, comes
        // back as `Some(0)` rather than being mistaken for unavailable.
        assert_relative_eq!(p.epm_batt_v().unwrap(), 12.6, epsilon = 0.01);
        let rails = p.epm_rail_ma();
        assert_relative_eq!(rails[0].unwrap() as f32, 120.0, epsilon = 40.0);
        assert_relative_eq!(rails[1].unwrap() as f32, 340.0, epsilon = 40.0);
        assert_eq!(rails[2], None);
        assert_relative_eq!(rails[3].unwrap() as f32, 780.0, epsilon = 40.0);
        assert_relative_eq!(rails[4].unwrap() as f32, 1500.0, epsilon = 40.0);
        assert_relative_eq!(rails[5].unwrap() as f32, 2400.0, epsilon = 40.0);
        let steps = p.sem_actuator_steps();
        assert_eq!(steps[0], Some(0));
        assert_relative_eq!(steps[1].unwrap() as f32, 1200.0, epsilon = 64.1);
        assert_relative_eq!(steps[2].unwrap() as f32, 34567.0, epsilon = 64.1);

        // The stack flags are individual packet fields now, not a relayed
        // 11 bit blob.
        assert!(p.payload_epm_alive());
        assert!(!p.payload_sem_alive());
        assert!(p.payload_exp2_active());
    }

    /// Absence has to survive the wire. Everything with a validity bit or a
    /// sentinel must come back as `None`, never as the 0.0 filler the encoder
    /// puts in the unused bits.
    #[test]
    fn absent_fields_round_trip_as_none() {
        init_logger();

        let p = round_trip(packet_with_nothing());

        assert_eq!(p.lat_lon(), None);
        assert_eq!(p.deployment_kf(), None);
        assert_eq!(p.deployment_kf_altitude_agl(), None);
        assert_eq!(p.deployment_kf_vertical_velocity(), None);
        assert_eq!(p.max_deployment_kf_altitude_agl(), None);
        assert_eq!(p.airbrakes_kf_tilt_deg(), None);
        assert_eq!(p.mpc_predicted_apogee_agl(), None);
        assert_eq!(p.icarus_air_brakes(), None);
        assert_eq!(p.air_brakes_actual_extension_percentage(), None);
        assert_eq!(p.air_brakes_servo_temp(), None);
        assert_eq!(p.epm_batt_v(), None);
        assert_eq!(p.epm_rail_ma(), [None; 6]);
        assert_eq!(p.sem_actuator_steps(), [None; 3]);

        // Fields that are always present stay present.
        assert_relative_eq!(p.vl_battery_v(), 7.4, epsilon = 0.01);
        assert_relative_eq!(p.air_temperature(), 25.5, epsilon = 0.2);
        assert_eq!(p.flight_stage(), FlightStage::Armed);
    }

    /// A NaN out of an estimator or the MPC solver used to panic inside
    /// `to_fixed_point_capped` (the min/max comparisons are both false for
    /// NaN, so it reached a float-to-int cast with no answer). It is absence
    /// that lost its `Option`, so it must land in the validity bit.
    #[test]
    fn nan_readings_become_absent_instead_of_panicking() {
        init_logger();

        let p = TelemetryPacket::new(
            10,
            true,
            12,
            Some((45.5, -73.6)),
            7.4,
            25.5,
            true,
            true,
            Some(DeploymentKfState {
                altitude_agl: f32::NAN,
                vertical_velocity: 0.0,
            }),
            Some(f32::NAN),
            Some(f32::NAN),
            FlightStage::Ascent,
            true,
            true,
            Some(f32::NAN),
            3000.0,
            true,
            false,
            8.2,
            false,
            PowerOutputStatus::PowerGood,
            false,
            PowerOutputStatus::PowerGood,
            false,
            PowerOutputStatus::Disabled,
            true,
            false,
            0.5,
            Some(IcarusAirBrakesState {
                actual_extension_percentage: f32::NAN,
                servo_temp: f32::NAN,
            }),
            false,
            false,
            false,
            false,
            PayloadSDRMCustomStatus::new(),
            None,
            [None; 6],
            [None; 3],
        );

        assert_eq!(p.deployment_kf(), None);
        assert_eq!(p.max_deployment_kf_altitude_agl(), None);
        assert_eq!(p.airbrakes_kf_tilt_deg(), None);
        assert_eq!(p.mpc_predicted_apogee_agl(), None);
        assert_eq!(p.icarus_air_brakes(), None);
    }

    /// The three payload fields spend their all-ones code on absence, so a
    /// real reading must never encode to it -- otherwise a rail pinned at its
    /// design maximum would report itself as unreadable, which is the one
    /// moment the ground most needs the number.
    #[test]
    fn full_scale_payload_readings_do_not_collide_with_the_sentinel() {
        init_logger();

        let p = TelemetryPacket::new(
            10,
            true,
            12,
            Some((45.5, -73.6)),
            7.4,
            25.5,
            true,
            true,
            None,
            None,
            None,
            FlightStage::Ascent,
            false,
            false,
            None,
            3000.0,
            false,
            false,
            8.2,
            false,
            PowerOutputStatus::Disabled,
            false,
            PowerOutputStatus::Disabled,
            false,
            PowerOutputStatus::Disabled,
            false,
            false,
            0.0,
            None,
            false,
            false,
            false,
            false,
            PayloadSDRMCustomStatus::new(),
            // Above the top of every range, so each one caps.
            Some(u16::MAX),
            [Some(u16::MAX); 6],
            [Some(u16::MAX); 3],
        );

        // One quantum below full scale, and emphatically not `None`.
        assert_relative_eq!(p.epm_batt_v().unwrap(), 16.9917, epsilon = 0.001);
        for rail in p.epm_rail_ma() {
            assert_eq!(rail, Some(4961));
        }
        for steps in p.sem_actuator_steps() {
            assert_eq!(steps, Some(65471));
        }

        // A genuine zero is still a zero, at the other end of the range.
        let p = round_trip(packet_with_everything(12));
        assert_eq!(p.sem_actuator_steps()[0], Some(0));
    }

    /// The satellite count is 5 bits, so packed_struct truncates rather than
    /// clamps. Without the saturation in `new`, 32 satellites downlink as 0 --
    /// the reading that means "no fix, do not fly". Anything at or above the
    /// cap must read as the cap, never wrap.
    #[test]
    fn satellite_count_saturates_instead_of_wrapping() {
        init_logger();

        for n in 0..=MAX_REPORTED_FIX_SATELLITES {
            assert_eq!(packet_with_everything(n).num_of_fix_satellites(), n);
        }
        for n in [
            MAX_REPORTED_FIX_SATELLITES + 1,
            MAX_REPORTED_FIX_SATELLITES + 2,
            40,
            99,
            u8::MAX,
        ] {
            assert_eq!(
                packet_with_everything(n).num_of_fix_satellites(),
                MAX_REPORTED_FIX_SATELLITES,
                "{} satellites must saturate, not wrap",
                n
            );
        }
    }

    /// The running maximum is latched: it must survive the estimator going
    /// absent, because the `Landed` packet is where the ground reads the
    /// apogee off, and by then the instantaneous altitude is `None`.
    #[test]
    fn max_altitude_latches_across_an_absent_estimator() {
        init_logger();

        let builder = TelemetryPacketBuilder::<embassy_sync::blocking_mutex::raw::NoopRawMutex>::new();

        // Nothing has been measured yet, so there is no maximum to report.
        assert_eq!(builder.create_packet().max_deployment_kf_altitude_agl(), None);

        builder.update(|state| {
            state.deployment_kf = Some(DeploymentKfState {
                altitude_agl: 1500.0,
                vertical_velocity: 100.0,
            });
        });
        builder.update(|state| {
            state.deployment_kf = Some(DeploymentKfState {
                altitude_agl: 900.0,
                vertical_velocity: -50.0,
            });
        });
        // The estimator is retired, the way it is after touchdown.
        builder.update(|state| {
            state.deployment_kf = None;
        });

        let p = builder.create_packet();
        assert_eq!(p.deployment_kf(), None);
        assert_relative_eq!(
            p.max_deployment_kf_altitude_agl().unwrap(),
            1500.0,
            epsilon = 0.7
        );
    }
}
