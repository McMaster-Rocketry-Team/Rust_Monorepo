use air_brakes_controller_core::airbrakes_estimator::{AirbrakesConfig, MachLockoutConfig};
use air_brakes_controller_core::{
    AirBrakesMPC, DeploymentProfile, FlightConfig, FlightEstimators, FlightProfile, ImuSample,
    RocketParameters,
};
use nalgebra::{Vector2, Vector3};

/// Stateless one-shot solve: the export the OpenRocket plugin has always
/// called, and the only one that existed before the harness below.
///
/// The horizontal velocity argument is NOT new fat — the plugin has been
/// passing it since it was written, and this export took ten arguments
/// while the plugin sent eleven, so every call through it trapped in
/// Chicory before reaching any Rust. The core solver has taken a
/// `Vector2` all along; this signature is what the caller already spoke.
#[unsafe(no_mangle)]
pub extern "C" fn get_air_brakes_extension_percentage(
    burnout_mass: f32,
    cd_0: f32,
    cd_25: f32,
    cd_50: f32,
    cd_75: f32,
    cd_100: f32,
    reference_area: f32,
    target_apogee_asl: f32,
    current_altitude_asl: f32,
    current_horizontal_velocity: f32,
    current_vertical_velocity: f32,
) -> f32 {
    let rocket_parameters = RocketParameters {
        burnout_mass,
        cd: [cd_0, cd_25, cd_50, cd_75, cd_100],
        reference_area,
    };

    AirBrakesMPC::new(rocket_parameters, target_apogee_asl)
        .update(
            current_altitude_asl,
            Vector2::new(current_horizontal_velocity, current_vertical_velocity),
        )
        .extension_percentage
}

// ---------------------------------------------------------------------------
// Full-loop harness exports: the SAME `FlightEstimators` + `AirBrakesMPC` the
// board runs, driven sample by sample from a host simulator.
//
// One module instance holds one flight, exactly like one boot of the
// firmware holds one flight. Chicory gives every simulation its own
// `Instance`, so these statics are per-flight state and not shared: there is
// no thread here, and no second flight inside one instance.
// ---------------------------------------------------------------------------

static mut ESTIMATORS: Option<FlightEstimators> = None;
static mut ROCKET: Option<RocketParameters> = None;
static mut MPC: Option<AirBrakesMPC> = None;
static mut LAST_PREDICTED_APOGEE_ASL: f32 = f32::NAN;

#[allow(static_mut_refs)]
fn estimators() -> Option<&'static mut FlightEstimators> {
    unsafe { ESTIMATORS.as_mut() }
}

/// Build the flight estimators from a flight config, field for field. The
/// host passes the numbers so that a sweep can vary them; `VLF5`'s
/// `FLIGHT_CONFIG` is what it passes for the nominal case.
///
/// `mach_lockout_duration_us` and `earliest/force` are `f32` seconds rather
/// than integer microseconds purely so every argument crossing the ABI is
/// one 32-bit float; they are converted here, not stored as floats.
#[unsafe(no_mangle)]
pub extern "C" fn harness_init(
    ignition_detection_acc_threshold: f32,
    // < 0 -> None
    deployment_mach_lockout_s: f32,
    // 0 -> Dual, 1 -> Single
    deployment_kind: i32,
    deployment_a: f32,
    deployment_b: f32,
    deployment_c: f32,
    deployment_d: f32,
    // < 0 -> None
    airbrakes_earliest_subsonic_s: f32,
    airbrakes_force_birth_s: f32,
    airbrakes_crossing_altitude_asl: f32,
    max_open_mach: f32,
    burnout_mass: f32,
    cd_0: f32,
    cd_25: f32,
    cd_50: f32,
    cd_75: f32,
    cd_100: f32,
    reference_area: f32,
) {
    let rocket = RocketParameters {
        burnout_mass,
        cd: [cd_0, cd_25, cd_50, cd_75, cd_100],
        reference_area,
    };

    let deployment = if deployment_kind == 0 {
        DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: deployment_a,
            drogue_chute_delay_us: (deployment_b * 1e6) as u32,
            main_chute_altitude_agl: deployment_c,
            main_chute_delay_us: (deployment_d * 1e6) as u32,
        }
    } else {
        DeploymentProfile::Single {
            minimum_deployment_altitude_agl: deployment_a,
            delay_us: (deployment_b * 1e6) as u32,
        }
    };

    let mach_lockout = if airbrakes_earliest_subsonic_s < 0.0 {
        None
    } else {
        Some(MachLockoutConfig {
            earliest_subsonic_after_ignition_us: (airbrakes_earliest_subsonic_s * 1e6) as u32,
            force_birth_after_ignition_us: (airbrakes_force_birth_s * 1e6) as u32,
            subsonic_crossing_altitude_asl: airbrakes_crossing_altitude_asl,
        })
    };

    let config = FlightConfig {
        ignition_detection_acc_threshold,
        profile: FlightProfile {
            mach_lockout_duration_us: if deployment_mach_lockout_s < 0.0 {
                None
            } else {
                Some((deployment_mach_lockout_s * 1e6) as u32)
            },
            deployment,
        },
        airbrakes: AirbrakesConfig {
            mach_lockout,
            max_open_mach,
            rocket: rocket.clone(),
        },
    };

    unsafe {
        ESTIMATORS = Some(FlightEstimators::new(config));
        ROCKET = Some(rocket);
        MPC = None;
        LAST_PREDICTED_APOGEE_ASL = f32::NAN;
    }
}

/// One sensor sample. `time_s` is the sample's timestamp on the host's
/// monotonic clock; the estimators measure every dt from it, so an
/// irregular feed is integrated honestly rather than assumed away.
///
/// Returns a bit field:
///   bit 0  MPC states available (the brakes are permitted to be open)
///   bit 1  the airbrakes half has been retired for good
///   bit 2  pad calibration complete
///   bit 3  burnout latched
///   bit 4  a drogue command was issued on this sample
///   bit 5  a main command was issued on this sample
///   bits 8..9  the airbrakes state (0 Armed, 1 Stage1, 2 DeadReckoning,
///              3 AirbrakesEnabled)
#[unsafe(no_mangle)]
pub extern "C" fn harness_update(
    time_s: f64,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    baro_altitude_asl: f32,
) -> i32 {
    let Some(estimators) = estimators() else {
        return 0;
    };

    let imu = ImuSample {
        acc: Vector3::new(acc_x, acc_y, acc_z),
        gyro: Vector3::new(gyro_x, gyro_y, gyro_z),
    };
    let (pyro, log) = estimators.update((time_s * 1e6) as u64, Some(&imu), baro_altitude_asl);

    let mut flags = 0i32;
    if estimators.airbrakes_mpc_states().is_some() {
        flags |= 1;
    }
    if estimators.airbrakes_estimator().is_none() {
        flags |= 2;
    }
    if let Some(airbrakes) = log.airbrakes {
        if airbrakes.calibration_complete {
            flags |= 4;
        }
        if airbrakes.burnout_detected {
            flags |= 8;
        }
        flags |= (airbrakes.state as i32 & 0x3) << 8;
    }
    match pyro {
        Some(firmware_common_new::vlp::packets::fire_pyro::PyroSelect::PyroDrogue) => flags |= 16,
        Some(firmware_common_new::vlp::packets::fire_pyro::PyroSelect::PyroMain) => flags |= 32,
        _ => {}
    }
    flags
}

/// The pad reference the MPC's AGL target is measured from.
#[unsafe(no_mangle)]
pub extern "C" fn harness_launch_pad_altitude_asl() -> f32 {
    estimators().map_or(f32::NAN, |e| e.launch_pad_altitude_asl())
}

/// The airbrakes filter's altitude ASL, or NaN while it has no reading.
#[unsafe(no_mangle)]
pub extern "C" fn harness_estimated_altitude_asl() -> f32 {
    estimators()
        .and_then(|e| e.airbrakes_mpc_states())
        .map_or(f32::NAN, |s| s.altitude_asl)
}

#[unsafe(no_mangle)]
pub extern "C" fn harness_estimated_vertical_velocity() -> f32 {
    estimators()
        .and_then(|e| e.airbrakes_mpc_states())
        .map_or(f32::NAN, |s| s.velocity.y)
}

#[unsafe(no_mangle)]
pub extern "C" fn harness_estimated_horizontal_velocity() -> f32 {
    estimators()
        .and_then(|e| e.airbrakes_mpc_states())
        .map_or(f32::NAN, |s| s.velocity.x)
}

#[unsafe(no_mangle)]
pub extern "C" fn harness_estimated_tilt_rad() -> f32 {
    estimators()
        .and_then(|e| e.airbrakes_estimator())
        .and_then(|ab| ab.tilt())
        .unwrap_or(f32::NAN)
}

/// Latch the MPC's target, once, exactly as `armed_mode` does: the pad
/// reference the deployment half is holding plus the configured AGL.
#[unsafe(no_mangle)]
pub extern "C" fn harness_mpc_latch_target(target_apogee_agl: f32) -> f32 {
    let Some(estimators) = estimators() else {
        return f32::NAN;
    };
    let target_asl = estimators.launch_pad_altitude_asl() + target_apogee_agl;
    #[allow(static_mut_refs)]
    unsafe {
        MPC = Some(AirBrakesMPC::new(
            ROCKET.clone().expect("harness_init first"),
            target_asl,
        ));
    }
    target_asl
}

/// One 10 Hz control tick against the estimator's own state. Returns the
/// commanded extension, or -1.0 when the gate is shut (which firmware
/// treats as a commanded 0.0 with no prediction).
#[unsafe(no_mangle)]
pub extern "C" fn harness_mpc_tick() -> f32 {
    let Some(estimators) = estimators() else {
        return -1.0;
    };
    let Some(states) = estimators.airbrakes_mpc_states() else {
        unsafe { LAST_PREDICTED_APOGEE_ASL = f32::NAN };
        return -1.0;
    };
    #[allow(static_mut_refs)]
    let mpc = unsafe { MPC.as_ref() };
    let Some(mpc) = mpc else {
        return -1.0;
    };
    let solution = mpc.update(states.altitude_asl, states.velocity);
    unsafe { LAST_PREDICTED_APOGEE_ASL = solution.predicted_apogee_asl };
    solution.extension_percentage
}

#[unsafe(no_mangle)]
pub extern "C" fn harness_mpc_predicted_apogee_asl() -> f32 {
    unsafe { LAST_PREDICTED_APOGEE_ASL }
}
