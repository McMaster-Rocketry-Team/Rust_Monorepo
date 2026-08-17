use crate::{
    controller::{DT, Derivative, RocketParameters, State},
    utils::approximate_air_density,
};

/// 2D ballistic dynamics: drag on total speed, opposing the velocity
/// vector; gravity on the vertical component.
pub fn calculate_state_derivatives(
    air_brakes_drag_percentage: f32,
    state: &State,
    rocket_param: &RocketParameters,
) -> Derivative<State> {
    let air_density = approximate_air_density(state.altitude_asl);

    let speed_squared = state.velocity.magnitude_squared();
    let cd = rocket_param.get_cd_from_drag_percentage(air_brakes_drag_percentage);
    // Drag acceleration is -(v/|v|) * k*|v|^2, which is just -k*|v|*v. Written
    // that way it costs one sqrt and a scalar multiply; written with
    // `normalize()` it cost a sqrt plus two divides, and nalgebra's sqrt comes
    // from `libm` (see `utils::sqrt`) rather than the FPU. Same arithmetic,
    // and on this M7 the pair was worth ~1.7 ms of a solve.
    let k = 0.5 * cd * air_density * rocket_param.reference_area / rocket_param.burnout_mass;
    let mut acceleration = if speed_squared > 1e-6 {
        state.velocity * (-k * crate::utils::sqrt(speed_squared))
    } else {
        nalgebra::Vector2::zeros()
    };
    acceleration.y -= 9.81;

    Derivative(State {
        altitude_asl: state.velocity.y,
        velocity: acceleration,
    })
}

/// Hard cap on the integration walk. The loop's only physical exit is
/// v_y <= 0, which gravity alone guarantees within v_y/9.81 s (drag only
/// shortens that), so this cap can never be reached by a trajectory the
/// model can actually fly: the worst case anywhere in the test suite,
/// including every Osiris replay, is 221 steps (22.1 s of coast at
/// DT = 0.1 s), and a vacuum ballistic coast from 400 m/s — faster than
/// this airframe flies — is 408. 2000 steps is 200 s of simulated coast,
/// ~9x the measured worst case. It exists purely so a numerical path the
/// finiteness check below does not catch still terminates.
const MAX_APOGEE_STEPS: usize = 2000;

// use rk2 to simulate the rocket until apogee
// apogee is when the vertical velocity <= 0
// in the first timestep, use first_tick_air_brakes_extension
// in all the following timestep, use 0.0 as air brakes extension
// returns the apogee altitude ASL (m)
pub fn simulate_apogee_rk2(
    first_tick_air_brakes_drag_percentage: f32,
    initial_state: &State,
    rocket_param: &RocketParameters,
) -> f32 {
    // A non-finite entry state has no trajectory to fly, and the normal exit
    // path would hand the caller `NaN + delta_alt` = NaN. That is the one
    // return value worth ruling out everywhere in this function: `NaN >
    // target` is false at every bisection step in `AirBrakesMPC::update`, the
    // NaN survives into `drag_percentage_to_extension_percentage`, and that
    // table walk falls through to 1.0 — full flap deploy off a garbage state
    // (measured pre-guard: vx = inf in, extension 1.0 out).
    //
    // 0 m ASL is below every reachable target apogee, so the bisection reads
    // this as an undershoot and stows instead.
    if !initial_state.altitude_asl.is_finite()
        || !initial_state.velocity.x.is_finite()
        || !initial_state.velocity.y.is_finite()
    {
        return 0.0;
    }

    // If we are already descending or stationary, return current altitude
    if initial_state.velocity.y <= 0.0 {
        return initial_state.altitude_asl;
    }

    let mut state = initial_state.clone();

    for step_index in 0..MAX_APOGEE_STEPS {
        let air_brakes_drag_percentage = match step_index {
            0 => first_tick_air_brakes_drag_percentage,
            1 => first_tick_air_brakes_drag_percentage / 2.0,
            _ => 0.0,
        };

        // RK2 (midpoint) integration
        let Derivative(k1) =
            calculate_state_derivatives(air_brakes_drag_percentage, &state, rocket_param);

        let mid_state = State {
            altitude_asl: state.altitude_asl + k1.altitude_asl * (0.5 * DT),
            velocity: state.velocity + k1.velocity * (0.5 * DT),
        };

        let Derivative(k2) =
            calculate_state_derivatives(air_brakes_drag_percentage, &mid_state, rocket_param);

        let next_state = State {
            altitude_asl: state.altitude_asl + k2.altitude_asl * DT,
            velocity: state.velocity + k2.velocity * DT,
        };

        // Divergence guard. The drag term is stiff — it grows with the square
        // of the speed — and at DT = 0.1 s a large enough horizontal velocity
        // makes the explicit integration blow up to inf and then NaN within a
        // handful of steps. `NaN <= 0.0` is false, so without this check the
        // apogee test below never fires and the loop runs to the step cap
        // (before the cap existed, forever: measured, vx = 458196 m/s hung a
        // 3 s watchdog).
        //
        // Hand back the *entry* altitude, not `state.altitude_asl`: by the
        // time the state goes non-finite the propagated altitude may have run
        // off to 1e30, which the caller would read as an enormous overshoot
        // and answer with full deploy — exactly the wrong way round. The entry
        // altitude means "no apogee above here can be predicted", the same
        // answer the already-descending case above gives, and it sits below
        // any reachable target so the bisection stows the flaps.
        if !next_state.altitude_asl.is_finite()
            || !next_state.velocity.x.is_finite()
            || !next_state.velocity.y.is_finite()
        {
            return initial_state.altitude_asl;
        }

        // Check for apogee crossing within this step
        let vy0 = state.velocity.y;
        let vy1 = next_state.velocity.y;
        if vy1 <= 0.0 {
            // Linearly interpolate vertical velocity over the step to estimate
            // the exact time t_zero where v_y crosses zero, then integrate
            // velocity to get altitude at apogee.
            let denom = vy1 - vy0;
            if denom.abs() < core::f32::EPSILON {
                return next_state.altitude_asl.max(state.altitude_asl);
            }
            let t_zero = DT * (-vy0) / denom; // 0 <= t_zero <= DT
            let delta_alt = vy0 * t_zero + 0.5 * (denom / DT) * t_zero * t_zero;
            return state.altitude_asl + delta_alt;
        }

        state = next_state;
    }

    // Step cap exhausted: 200 s of simulated coast without v_y reaching zero.
    // No flyable trajectory gets here (see `MAX_APOGEE_STEPS`), so there is no
    // better answer available than the same "no apogee above here" the guards
    // above return.
    initial_state.altitude_asl
}

#[cfg(test)]
mod test {
    use nalgebra::Vector2;

    use crate::{controller::AirBrakesMPC, tests::init_logger};

    use super::*;

    /// The 2D sim must account for tilt: the same total speed with a
    /// horizontal component reaches a lower apogee than flying straight
    /// up.
    #[test]
    fn tilted_flight_reaches_lower_apogee() {
        init_logger();
        let rocket_param = RocketParameters {
            burnout_mass: 19.417,
            cd: [0.5; 5],
            reference_area: 0.0136,
        };
        let straight = simulate_apogee_rk2(
            0.0,
            &State {
                altitude_asl: 1000.0,
                velocity: Vector2::new(0.0, 250.0),
            },
            &rocket_param,
        );
        let tilted = simulate_apogee_rk2(
            0.0,
            &State {
                altitude_asl: 1000.0,
                // same total speed, 30 deg tilt
                velocity: Vector2::new(125.0, 216.5),
            },
            &rocket_param,
        );
        log_info!("straight {straight}, tilted {tilted}");
        assert!(tilted < straight - 100.0);
    }

    /// A pathological horizontal velocity must terminate, and must not come
    /// back out as a full-deploy command.
    ///
    /// Measured before the guards, on the host with a 3 s watchdog:
    /// vx = 458196 m/s never returned at all — the stiff drag term blows the
    /// explicit integration up to inf and then NaN, and `NaN <= 0.0` is
    /// false, so the apogee test never fires — and vx = inf returned a NaN
    /// apogee that `AirBrakesMPC::update` turned into extension 1.0, full
    /// flap deploy off a garbage state. On the flight computer the first is a
    /// hung MPC task and the second is the flaps out at max q.
    ///
    /// The only thing standing between the estimator and these inputs today
    /// is `TILT_CAP_RAD` clamping the tilt, which is a coincidence of the
    /// current tuning, not a guard.
    ///
    /// Each case runs on its own thread so a regression fails this test
    /// instead of hanging the suite.
    #[test]
    fn pathological_horizontal_velocity_terminates_and_stows() {
        init_logger();

        // Strictly increasing cd table so the extension mapping is actually
        // exercised; a flat table short-circuits to 0.0 on its first branch
        // and would hide a bad answer.
        let rocket_param = RocketParameters {
            burnout_mass: 19.417,
            cd: [0.3, 0.4, 0.5, 0.65, 0.8],
            reference_area: 0.0136,
        };
        const ALT_ASL: f32 = 1032.0 + 251.0;
        const TARGET_ASL: f32 = 3048.0; // 10000 ft, Osiris's target

        // 458196: the measured hang. inf/NaN: the measured full deploy.
        // 1e30: squares to inf inside `magnitude_squared`, so the state goes
        // non-finite on the very first step rather than after a few.
        for vx in [458196.0f32, f32::INFINITY, f32::NAN, 1e30f32] {
            let (tx, rx) = std::sync::mpsc::channel();
            let param = rocket_param.clone();
            std::thread::spawn(move || {
                let apogee = simulate_apogee_rk2(
                    0.5,
                    &State {
                        altitude_asl: ALT_ASL,
                        velocity: Vector2::new(vx, 308.7624),
                    },
                    &param,
                );
                let solution = AirBrakesMPC::new(param, TARGET_ASL)
                    .update(ALT_ASL, Vector2::new(vx, 308.7624));
                let _ = tx.send((apogee, solution));
            });

            let (apogee, solution) = rx
                .recv_timeout(std::time::Duration::from_secs(3))
                .unwrap_or_else(|_| panic!("simulate_apogee_rk2 did not return in 3 s at vx={vx}"));
            log_info!("vx={vx}: apogee {apogee}, {solution:?}");

            // A non-finite apogee is what feeds the full-deploy fall-through.
            assert!(apogee.is_finite(), "vx={vx}: apogee {apogee} not finite");
            assert!(
                solution.predicted_apogee_asl.is_finite(),
                "vx={vx}: predicted apogee {} not finite",
                solution.predicted_apogee_asl
            );
            // The bail-out is at or below the current altitude, so the MPC has
            // to read it as an undershoot and stow. With this cd table the
            // bisection lands on drag -0.875, i.e. extension 0.078.
            assert!(
                apogee <= ALT_ASL,
                "vx={vx}: bail-out apogee {apogee} is above the current altitude"
            );
            assert!(
                solution.extension_percentage < 0.1,
                "vx={vx}: commanded extension {} on a garbage state",
                solution.extension_percentage
            );
        }
    }
}
