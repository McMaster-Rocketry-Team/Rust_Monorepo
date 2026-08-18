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
    calculate_state_derivatives_at_cd(
        rocket_param.get_cd_from_drag_percentage(air_brakes_drag_percentage),
        state,
        rocket_param,
    )
}

/// The same dynamics with cd already resolved.
///
/// [`simulate_apogee_rk2`] holds cd fixed across both stages of an RK2 step and
/// changes it only while the modelled servo is moving -- a handful of steps out
/// of a coast that runs to a couple of hundred -- so resolving it per step
/// instead of per derivative evaluation halves the interpolation work on the
/// hot path and lets the tail run with no interpolation at all.
fn calculate_state_derivatives_at_cd(
    cd: f32,
    state: &State,
    rocket_param: &RocketParameters,
) -> Derivative<State> {
    let air_density = approximate_air_density(state.altitude_asl);

    let speed_squared = state.velocity.magnitude_squared();
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

/// How many 0.1 s steps the solver holds its candidate before letting the
/// modelled servo retract to the neutral tail.
///
/// FDR section 16.7.4.2's rule 1 is a single tick, which is the right model for
/// an instantaneous actuator. Icarus's servo is not one. Measured off both
/// transitions in VLF5's HIL log -- the deploy at 236.534 s and the retract at
/// 258.336 s, which agree to 1% -- it takes 25.8 ms of dead time and then
/// 0.376 s of slew at 2.67 /s to cross the full stroke: 0.40 s, four MPC
/// ticks. A command issued now is therefore still in transit four ticks later,
/// and the receding horizon re-issues it on every one of them. Holding the
/// candidate for the stroke time is what the plant does with a command; holding
/// it for one tick is what nothing does.
///
/// Against the one-tick hold, simulated on the rate-limited plant from the
/// LC'25 birth state: apogee tracking is unchanged -- 0.00 m at every reachable
/// target tried, with the plant's cd perturbed +/-10% and from a late gate --
/// commanded servo travel over a flight falls about 30%, and the spread between
/// the bisection's two rails widens from 10.3 m to 30.2 m, which is the signal
/// the final interpolation resolves the command out of.
///
/// Feeding the servo's *measured* extension into the prediction was tried
/// instead and rejected: it closes a loop from servo position to command that
/// the one-tick horizon cannot damp, and multiplied commanded travel by 10-20x
/// for no tracking gain.
const CANDIDATE_HOLD_STEPS: usize = 4;

/// Extension the modelled servo covers in one step on its way back to neutral.
///
/// The full stroke is [`CANDIDATE_HOLD_STEPS`] steps by definition of that
/// constant, so one step is that fraction of it. This makes the handover as
/// long as the travel demands rather than a fixed number of steps: from full
/// deploy the flaps reach neutral in two steps, from stowed in three.
const RETRACT_PER_STEP: f32 = 1.0 / CANDIDATE_HOLD_STEPS as f32;

/// RK2 the rocket to apogee -- the first step where v_y <= 0 -- and return
/// that altitude, ASL (m).
///
/// The schedule below is FDR section 16.7.4.2's optimal extension sequence, and
/// every number in it is a *drag* percentage (-1 stowed, +1 full), never an
/// extension percentage. The two are different axes; see
/// [`RocketParameters::get_cd_from_drag_percentage`].
///
/// - steps 0..CANDIDATE_HOLD_STEPS: the candidate the solver is testing
///   (the report's rule 1, stretched to the servo's stroke time -- see that
///   constant)
/// - then the flaps retract to neutral at the servo's rate, in *extension*,
///   which is where the rate limit physically lives (rule 3's intermediate
///   ticks, as many as the travel needs rather than a single half-drag step)
/// - once neutral: drag 0.0, held all the way to apogee (rule 2)
///
/// Drag 0.0 is **not** stowed. It is the neutral position -- half the brakes'
/// full drag contribution, ~60% extension on VLF5's cd table -- and the report
/// picks it deliberately: parked mid-authority, a later disturbance can be
/// answered by adding drag or by giving drag back, in equal measure. A stowed
/// tail has nothing to give back, and a tail above neutral has less room to
/// brake; both are rejected by name in the sequence table.
///
/// Two consequences to know before reading the number this returns:
/// - it means "apogee if I fly neutral from here", which is the report's
///   *nominal apogee*, not a forecast of where the rocket ends up;
/// - only the held steps differ between candidates, so sweeping the whole
///   [-1, +1] range moves the answer ~30 m against ~330 m of real brake
///   authority. The loop closes by re-solving every tick, not by this
///   gradient.
pub fn simulate_apogee_rk2(
    candidate_air_brakes_drag_percentage: f32,
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

    // The schedule, carried as an extension rather than as a drag percentage.
    // The retract is rate limited, and the rate limit lives in the linkage, so
    // it is linear in extension and not in drag: ramping the drag percentage
    // instead would walk the flaps back along the wrong curve, slowly at the
    // deployed end where cd changes fastest. Both coordinates agree at the
    // ends, so the held steps are unchanged -- only the handover moves.
    let neutral_extension = rocket_param.neutral_extension_percentage();
    let mut extension = rocket_param
        .drag_percentage_to_extension_percentage(candidate_air_brakes_drag_percentage);
    let mut cd = rocket_param.get_cd_from_extension_percentage(extension);

    for step_index in 0..MAX_APOGEE_STEPS {
        // RK2 (midpoint) integration
        let Derivative(k1) = calculate_state_derivatives_at_cd(cd, &state, rocket_param);

        let mid_state = State {
            altitude_asl: state.altitude_asl + k1.altitude_asl * (0.5 * DT),
            velocity: state.velocity + k1.velocity * (0.5 * DT),
        };

        let Derivative(k2) = calculate_state_derivatives_at_cd(cd, &mid_state, rocket_param);

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

        // Step the servo for the next step. Exact assignment on arrival, so
        // the comparison above stops the interpolation for good rather than
        // creeping on float residue.
        if step_index + 1 >= CANDIDATE_HOLD_STEPS && extension != neutral_extension {
            let remaining = neutral_extension - extension;
            extension = if remaining.abs() <= RETRACT_PER_STEP {
                neutral_extension
            } else if remaining > 0.0 {
                extension + RETRACT_PER_STEP
            } else {
                extension - RETRACT_PER_STEP
            };
            cd = rocket_param.get_cd_from_extension_percentage(extension);
        }
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

    use approx::assert_relative_eq;

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

    /// VLF5's hil-dual airframe at the LC'25 airbrakes birth state.
    fn vlf5() -> (RocketParameters, State) {
        (
            RocketParameters {
                burnout_mass: 18.696,
                cd: [0.61365, 0.69816, 0.8084, 0.96641, 1.12441],
                reference_area: 0.009854945,
            },
            State {
                altitude_asl: 6802.0474,
                velocity: Vector2::new(31.1866, 244.46007),
            },
        )
    }

    /// The bisection in `AirBrakesMPC::update` is only valid because apogee
    /// falls monotonically as the candidate's drag rises -- FDR section
    /// 16.7.4.3 rests the whole solver on it. Holding the candidate for the
    /// servo stroke rather than one tick must not break that.
    #[test]
    fn apogee_is_strictly_decreasing_in_drag_percentage() {
        init_logger();
        let (rocket, state) = vlf5();

        let mut previous = f32::INFINITY;
        for i in 0..=40 {
            let drag = -1.0 + 2.0 * (i as f32) / 40.0;
            let apogee = simulate_apogee_rk2(drag, &state, &rocket);
            assert!(
                apogee < previous,
                "apogee {apogee} at drag {drag} did not fall below {previous}"
            );
            previous = apogee;
        }

        let spread =
            simulate_apogee_rk2(-1.0, &state, &rocket) - simulate_apogee_rk2(1.0, &state, &rocket);
        log_info!("rail-to-rail spread {spread} m");
        // One tick of hold gave 10.3 m here, which is what the final
        // interpolation has to resolve a command out of. The stroke-length
        // hold roughly triples it.
        assert!(spread > 25.0, "rail-to-rail spread collapsed to {spread} m");
    }

    /// `predicted_apogee_asl` must describe the extension actually commanded.
    ///
    /// It used to clamp the drag percentage to [0, 1] -- the *extension*
    /// range applied to the drag axis -- so every command below neutral was
    /// reported at drag 0.0. Asking for stowed flaps came back with the
    /// apogee for ~60% flaps.
    #[test]
    fn predicted_apogee_belongs_to_the_commanded_extension() {
        init_logger();
        let (rocket, state) = vlf5();

        // Far above anything this coast can reach, so the solve stows.
        let solution = AirBrakesMPC::new(rocket.clone(), 20000.0)
            .update(state.altitude_asl, state.velocity);
        assert!(
            solution.extension_percentage < 1e-6,
            "expected a stowed command, got {}",
            solution.extension_percentage
        );

        let stowed = simulate_apogee_rk2(-1.0, &state, &rocket);
        let neutral = simulate_apogee_rk2(0.0, &state, &rocket);
        log_info!(
            "reported {}, stowed {stowed}, neutral {neutral}",
            solution.predicted_apogee_asl
        );
        assert_relative_eq!(solution.predicted_apogee_asl, stowed, epsilon = 1e-2);
        assert!(
            (solution.predicted_apogee_asl - neutral).abs() > 1.0,
            "reported apogee is still the neutral-tail number"
        );
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
