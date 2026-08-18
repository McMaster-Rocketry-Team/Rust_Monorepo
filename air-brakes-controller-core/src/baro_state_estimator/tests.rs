use super::*;
use crate::tests::fixtures::{IGNITION_ACC_THRESHOLD, subsonic_profile};

/// Deterministic pseudo-gaussian noise (sum of 4 LCG uniforms, mean 0)
struct NoiseGen {
    state: u32,
    std: f32,
}

impl NoiseGen {
    fn new(std: f32) -> Self {
        Self { state: 12345, std }
    }

    fn next(&mut self) -> f32 {
        let mut sum = 0.0f32;
        for _ in 0..4 {
            self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
            sum += (self.state >> 8) as f32 / (1 << 24) as f32 - 0.5;
        }
        // sum of 4 U(-0.5, 0.5) has std sqrt(4/12) = 0.577
        sum / 0.577 * self.std
    }
}

/// A monotonic sample clock at the nominal rate.
///
/// `update` takes a timestamp now, and most of the tests below are about
/// the state machine rather than about timing, so they feed the nominal
/// tick and read exactly as they did when the machine counted samples.
/// Accumulating in nanoseconds keeps the tick exact — 1/416 s is not a
/// whole number of microseconds, and rounding it there would drift a
/// 48 s lockout by tens of milliseconds.
///
/// The tests that ARE about rate build their own timestamps; see
/// `timers_are_independent_of_the_sample_rate`.
struct SampleClock {
    ns: u64,
}

impl SampleClock {
    fn new() -> Self {
        Self { ns: 0 }
    }

    /// Timestamp (us) of this sample, then advance one nominal tick.
    fn tick(&mut self) -> u64 {
        let now = self.ns / 1000;
        self.ns += 1_000_000_000 / SAMPLES_PER_S as u64;
        now
    }
}

/// Specific force on a vertical airframe's axis, device +Z — the convention
/// `hil::osiris` replays and the one the estimator's magnitude check is
/// indifferent to anyway.
fn sf(specific_force: f32) -> Option<Vector3<f32>> {
    Some(Vector3::new(0.0, 0.0, specific_force))
}

/// What an accelerometer reads on the rail: the pad is holding the weight.
const PAD_SF: f32 = 9.81;

struct FlightResult {
    drogue: Option<(usize, f32)>,
    main: Option<(usize, f32)>,
    apogee_agl: f32,
    final_state: RocketState,
}

/// Simulates a simple vertical flight, feeding noisy baro altitude to the estimator.
fn simulate_flight(
    estimator: &mut RocketStateEstimator,
    pad_altitude_asl: f32,
    burn_acceleration: f32,
    burn_time_s: f32,
) -> FlightResult {
    let mut clock = SampleClock::new();
    let mut noise = NoiseGen::new(0.5);
    let mut drogue = None;
    let mut main = None;
    let mut sample_i = 0usize;

    let mut feed = |estimator: &mut RocketStateEstimator, altitude_asl: f32, specific_force: f32| {
        let (pyro, _gate) = estimator.update(clock.tick(), sf(specific_force), altitude_asl + noise.next());
        // Pyros only fire in chute stages, where the KF is live, so the
        // altitude is present on exactly the samples recorded below. A
        // `None` at a fire would mean a pyro went off inside the Mach
        // lockout — the one thing the lockout exists to make impossible —
        // so unwrapping it here is itself part of what these tests check.
        let kf_agl = estimator
            .kf_altitude_asl()
            .map(|altitude_asl| altitude_asl - estimator.launch_pad_altitude_asl());
        match pyro {
            Some(PyroSelect::PyroDrogue) => {
                assert!(drogue.is_none(), "drogue fired more than once");
                drogue = Some((sample_i, kf_agl.expect("no KF altitude at drogue fire")));
            }
            Some(PyroSelect::PyroMain) => {
                assert!(main.is_none(), "main fired more than once");
                main = Some((sample_i, kf_agl.expect("no KF altitude at main fire")));
            }
            None => {}
        }
        sample_i += 1;
    };

    // 30 s sitting on the pad
    for _ in 0..(30 * SAMPLES_PER_S) {
        feed(estimator, pad_altitude_asl, PAD_SF);
    }
    assert!(matches!(estimator.state(), RocketState::OnPad));

    // powered ascent + coast + free fall, simple point-mass integration
    let mut altitude_asl = pad_altitude_asl;
    let mut velocity = 0.0f32;
    let mut t = 0.0f32;
    let mut apogee_agl = 0.0f32;
    let descent_terminal_velocity = -25.0f32;
    loop {
        let acceleration = if t < burn_time_s {
            burn_acceleration
        } else {
            -9.81
        };
        velocity += acceleration * DT;
        if velocity < descent_terminal_velocity {
            velocity = descent_terminal_velocity;
        }
        altitude_asl += velocity * DT;
        t += DT;

        if altitude_asl <= pad_altitude_asl {
            break;
        }
        apogee_agl = apogee_agl.max(altitude_asl - pad_altitude_asl);
        // Thrust also carries the weight; ballistic coast senses nothing;
        // terminal descent is aerodynamically supported at 1 g.
        let specific_force = if t < burn_time_s {
            burn_acceleration + 9.81
        } else if velocity <= descent_terminal_velocity {
            9.81
        } else {
            0.0
        };
        feed(estimator, altitude_asl, specific_force);
    }

    // 30 s sitting on the ground
    for _ in 0..(30 * SAMPLES_PER_S) {
        feed(estimator, pad_altitude_asl, PAD_SF);
    }

    FlightResult {
        drogue,
        main,
        apogee_agl,
        final_state: estimator.state(),
    }
}

#[test]
fn dual_deploys_drogue_near_apogee_and_main_at_altitude() {
    let main_agl = 457.2;
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 500.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: main_agl,
                main_chute_delay_us: 0,
            },
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );

    let result = simulate_flight(&mut estimator, 200.0, 80.0, 3.0);
    assert!(result.apogee_agl > 1000.0, "apogee={}", result.apogee_agl);

    let (drogue_i, drogue_agl) = result.drogue.expect("expected drogue deploy");
    let (main_i, main_deploy_agl) = result.main.expect("expected main deploy");
    assert!(main_i > drogue_i);
    assert!(
        (drogue_agl - result.apogee_agl).abs() < 200.0,
        "drogue agl={} apogee={}",
        drogue_agl,
        result.apogee_agl
    );
    assert!(
        (main_deploy_agl - main_agl).abs() < 100.0,
        "main deploy agl={} expected ~{}",
        main_deploy_agl,
        main_agl
    );
    assert!(matches!(result.final_state, RocketState::Landed));
}

#[test]
fn single_deploys_both_near_apogee() {
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            // 500 m, against an apogee the test then asserts is over 1000 m:
            // the minimum has to be cleared for either chute to come out.
            deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 500.0,
                delay_us: 0,
            },
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );

    let result = simulate_flight(&mut estimator, 200.0, 80.0, 3.0);
    assert!(result.apogee_agl > 1000.0);

    let (drogue_i, drogue_agl) = result.drogue.expect("expected drogue deploy");
    let (main_i, main_agl) = result.main.expect("expected main deploy");
    assert_eq!(main_i, drogue_i + 1, "main should follow drogue on next sample");
    assert!(
        (drogue_agl - result.apogee_agl).abs() < 200.0,
        "drogue agl={} apogee={}",
        drogue_agl,
        result.apogee_agl
    );
    assert!(
        (main_agl - result.apogee_agl).abs() < 200.0,
        "main agl={} apogee={}",
        main_agl,
        result.apogee_agl
    );
    assert!(matches!(result.final_state, RocketState::Landed));
}

#[test]
fn below_min_apogee_does_not_deploy() {
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            // 5000 m against a flight that reaches nowhere near it — this
            // number is the whole test.
            deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 5000.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: 457.2,
                main_chute_delay_us: 0,
            },
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );

    let result = simulate_flight(&mut estimator, 200.0, 40.0, 1.5);
    assert!(result.drogue.is_none());
    assert!(result.main.is_none());
    assert!(matches!(
        result.final_state,
        RocketState::FailedToReachMinApogee
    ));
}

#[test]
fn no_false_ignition_on_pad() {
    let mut clock = SampleClock::new();
    // The rocket never leaves the pad, so no chute number can matter here.
    let mut estimator = RocketStateEstimator::new(subsonic_profile(), IGNITION_ACC_THRESHOLD);
    let mut noise = NoiseGen::new(0.5);

    // 2 minutes armed on the pad
    for _ in 0..(120 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), 200.0 + noise.next());
    }
    assert!(matches!(estimator.state(), RocketState::OnPad));
}

/// The pad reference cannot contain a reading from the last second before
/// the motor lights.
///
/// That is the whole reason `PadReference` hands its windows out one behind
/// instead of averaging up to ignition: whatever an igniter does to a static
/// port, and whatever the 0.1 s detection sustain costs on top of it, lands
/// in a window nobody reads. A disturbance shorter than `PAD_WINDOW_S` can
/// reach the window *waiting* to become the reference; it cannot reach the
/// reference. 50 m here rather than something subtle because the assertion
/// is 1 m wide — this test is about which samples are in the average, not
/// about how well an average rejects anything.
#[test]
fn pad_reference_excludes_the_second_before_ignition() {
    const PAD_ASL: f32 = 200.0;
    const DISTURBANCE_M: f32 = 50.0;
    const DISTURBANCE_S: f32 = 0.5;

    let mut clock = SampleClock::new();
    // Subsonic, so ignition goes straight to `Ascent` and freezes the
    // reference there; no chute number is read.
    let mut estimator = RocketStateEstimator::new(subsonic_profile(), IGNITION_ACC_THRESHOLD);
    let mut noise = NoiseGen::new(0.5);

    // A fifth of a second in — before any window has closed — there is
    // already a reference, and it is the pad rather than 0. Everything
    // downstream reads this as "absent means no estimator sample", so a
    // young session must not look like a pad at sea level.
    for _ in 0..(SAMPLES_PER_S / 5) {
        estimator.update(clock.tick(), sf(PAD_SF), PAD_ASL + noise.next());
    }
    let young = estimator.launch_pad_altitude_asl();
    assert!(
        (young - PAD_ASL).abs() < 1.0,
        "0.2 s in the reference read {young:.2}m against a true {PAD_ASL}m"
    );

    // 30 s of quiet rail.
    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), PAD_ASL + noise.next());
    }
    let quiet = estimator.launch_pad_altitude_asl();
    assert!(
        (quiet - PAD_ASL).abs() < 1.0,
        "quiet pad read {quiet:.2}m against a true {PAD_ASL}m"
    );

    // The half second before the motor lights, with the port reading 50 m out.
    for _ in 0..(DISTURBANCE_S * SAMPLES_PER_S as f32) as usize {
        estimator.update(
            clock.tick(),
            sf(PAD_SF),
            PAD_ASL + DISTURBANCE_M + noise.next(),
        );
    }

    // Motor lights, port still out for the whole 0.1 s the detector sustains
    // over. Bounded so a detector that never latches fails as a missing
    // ignition rather than as a hang.
    let mut i = 0;
    while matches!(estimator.state(), RocketState::OnPad) {
        estimator.update(
            clock.tick(),
            sf(20.0 * 9.81),
            PAD_ASL + DISTURBANCE_M + noise.next(),
        );
        i += 1;
        assert!(i < SAMPLES_PER_S, "ignition never latched at 20 g");
    }

    let latched = estimator.launch_pad_altitude_asl();
    eprintln!("pad reference at ignition {latched:.2}m, true pad {PAD_ASL}m");
    assert!(
        (latched - PAD_ASL).abs() < 1.0,
        "pad reference {latched:.2}m carries the {DISTURBANCE_M}m disturbance"
    );
}

/// Redundant-computer ejection blast during coast (60 ms of readings ~1400 m
/// low, injected 3 s before apogee): the innovation gate must reject every
/// sample and apogee detection must not fire early.
#[test]
fn blast_transient_during_coast_does_not_deploy_early() {
    let mut clock = SampleClock::new();
    // What is asserted is *when* apogee is latched relative to the true
    // apogee, not what altitude the chute comes out at.
    let mut estimator = RocketStateEstimator::new(subsonic_profile(), IGNITION_ACC_THRESHOLD);
    let mut noise = NoiseGen::new(0.5);
    let pad = 200.0f32;

    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), pad + noise.next());
    }

    // 2 s burn at 80 m/s^2 -> 160 m/s; apogee ~16.3 s after burnout
    let mut altitude_asl = pad;
    let mut velocity = 0.0f32;
    let mut t = 0.0f32;
    let mut apogee_i = 0usize;
    let mut apogee_agl = 0.0f32;
    let mut drogue_i = None;
    let mut i = 0usize;
    let blast_start_t = 2.0 + 160.0 / 9.81 - 3.0; // 3 s before apogee
    loop {
        let acceleration = if t < 2.0 { 80.0 } else { -9.81 };
        velocity += acceleration * DT;
        if velocity < -25.0 {
            velocity = -25.0;
        }
        altitude_asl += velocity * DT;
        t += DT;
        if altitude_asl <= pad {
            break;
        }
        if altitude_asl - pad > apogee_agl {
            apogee_agl = altitude_asl - pad;
            apogee_i = i;
        }

        let in_blast = t >= blast_start_t && t < blast_start_t + 0.06;
        let measured = if in_blast {
            altitude_asl - 1400.0
        } else {
            altitude_asl + noise.next()
        };
        let specific_force = if t < 2.0 {
            80.0 + 9.81
        } else if velocity <= -25.0 {
            9.81
        } else {
            0.0
        };
        let (pyro, _gate) = estimator.update(clock.tick(), sf(specific_force), measured);
        if let Some(PyroSelect::PyroDrogue) = pyro {
            assert!(drogue_i.is_none());
            drogue_i = Some((i, altitude_asl - pad));
        }
        i += 1;
    }

    let (drogue_i, drogue_agl) = drogue_i.expect("expected drogue deploy");
    assert!(
        drogue_i > apogee_i,
        "drogue fired before apogee: fired at sample {}, apogee at {}",
        drogue_i,
        apogee_i
    );
    assert!(
        (drogue_agl - apogee_agl).abs() < 150.0,
        "drogue agl={} apogee={}",
        drogue_agl,
        apogee_agl
    );
}

/// Mach 1.9 flight with shock-garbage baro while supersonic: the timed
/// lockout (started at ignition detection) must freeze the filter through the
/// garbage, re-seed on exit, and the flight must still deploy at apogee and
/// land normally.
#[test]
fn mach_lockout_survives_supersonic_garbage() {
    let mut clock = SampleClock::new();
    // Sim profile: 6.5 s burn at 100 m/s^2 -> 650 m/s (~Mach 1.9),
    // gravity-only coast, apogee ~72.8 s. Ignition detection completes ~1.5 s
    // into the burn; back below Mach 0.75 at ~43 s. Lockout = 48 s from
    // detection (~margin), ending ~23 s before apogee.
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            mach_lockout_duration_us: Some(48_000_000),
            deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 1000.0,
                delay_us: 0,
            },
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );
    let mut noise = NoiseGen::new(0.5);
    let pad = 200.0f32;

    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), pad + noise.next());
    }
    assert!(matches!(estimator.state(), RocketState::OnPad));

    let mut altitude_asl = pad;
    let mut velocity = 0.0f32;
    let mut t = 0.0f32;
    let mut apogee_agl = 0.0f32;
    let mut entered_lockout = false;
    let mut fired_in_lockout = false;
    let mut drogue_agl = None;
    let descent_terminal_velocity = -25.0f32;
    loop {
        let acceleration = if t < 6.5 { 100.0 } else { -9.81 };
        velocity += acceleration * DT;
        if velocity < descent_terminal_velocity {
            velocity = descent_terminal_velocity;
        }
        altitude_asl += velocity * DT;
        t += DT;
        if altitude_asl <= pad {
            break;
        }
        apogee_agl = apogee_agl.max(altitude_asl - pad);

        // Above ~Mach 0.85 the static port reads shock garbage: a large
        // offset plus 50x noise. An unprotected filter would track this.
        let measured = if velocity.abs() > 0.85 * 340.0 {
            altitude_asl - 800.0 + noise.next() * 50.0
        } else {
            altitude_asl + noise.next()
        };
        let specific_force = if t < 6.5 {
            100.0 + 9.81
        } else if velocity <= descent_terminal_velocity {
            9.81
        } else {
            0.0
        };
        let (pyro, _gate) = estimator.update(clock.tick(), sf(specific_force), measured);
        // While locked out the estimator must say so honestly: the reported
        // state is MachLockout carrying only the pad altitude — the frozen
        // KF altitude/velocity are not reachable through `state()` at all.
        if let RocketState::MachLockout {
            launch_pad_altitude_asl,
        } = estimator.state()
        {
            entered_lockout = true;
            fired_in_lockout |= pyro.is_some();
            assert!(
                (launch_pad_altitude_asl - pad).abs() < 5.0,
                "lockout pad asl={} expected ~{}",
                launch_pad_altitude_asl,
                pad
            );
            // The log path must say the same thing the state does: the
            // there is no filter at all here — it is dropped at ignition
            // and rebuilt when the lockout ends — so neither accessor hands
            // out a number. Were they to keep reporting a frozen one, the SD
            // log would disagree with the telemetry over this whole window.
            assert_eq!(estimator.kf_altitude_asl(), None);
            assert_eq!(estimator.kf_vertical_velocity(), None);
        } else if !matches!(estimator.state(), RocketState::OnPad) {
            // Once the lockout ends, a filter exists for the rest of the
            // flight. On the pad there is none either: nothing needs one
            // before the rocket moves.
            assert!(estimator.kf_altitude_asl().is_some());
            assert!(estimator.kf_vertical_velocity().is_some());
        }
        if matches!(pyro, Some(PyroSelect::PyroDrogue)) {
            drogue_agl = Some(altitude_asl - pad);
        }
    }

    assert!(entered_lockout, "lockout never engaged");
    assert!(!fired_in_lockout, "pyro fired during lockout");
    assert!(
        !matches!(estimator.state(), RocketState::MachLockout { .. }),
        "lockout never exited"
    );
    let drogue_agl = drogue_agl.expect("expected drogue deploy");
    assert!(
        (drogue_agl - apogee_agl).abs() < 150.0,
        "drogue agl={} apogee={}",
        drogue_agl,
        apogee_agl
    );

    // 10 s on the ground -> landed
    for _ in 0..(10 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), pad + noise.next());
    }
    assert!(matches!(estimator.state(), RocketState::Landed));
}

/// The two KF accessors report absence as absence. There are exactly two
/// ways to have no reading — the filter is not born yet (`new` does not
/// construct it; the first `update` does), and the Mach lockout has it
/// frozen — and both must come back `None` rather than as a zero that reads
/// like a measurement. Everything else, on-pad included, is a live filter
/// and must come back `Some`, even where the corresponding [`RocketState`]
/// variant carries no altitude of its own.
#[test]
fn kf_accessors_absent_before_birth_and_during_lockout() {
    let mut clock = SampleClock::new();
    // The 10 s lockout is one of the two ways to have no reading, so it is
    // stated here; the chute profile never comes into it.
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            mach_lockout_duration_us: Some(10_000_000),
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );

    assert_eq!(estimator.kf_altitude_asl(), None, "no filter before the first sample");
    assert_eq!(estimator.kf_vertical_velocity(), None, "no filter before the first sample");

    let mut noise = NoiseGen::new(0.5);
    let pad = 200.0f32;
    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), sf(PAD_SF), pad + noise.next());
    }

    // There is no filter on the pad either: half a minute of samples in, the
    // barometer has been tracking the pad altitude and nothing else. What the
    // log gets here is `launch_pad_altitude_asl`, which is the only thing the
    // barometer is actually estimating before the rocket moves.
    assert!(matches!(estimator.state(), RocketState::OnPad));
    assert_eq!(estimator.kf_altitude_asl(), None, "no filter on the pad");
    assert_eq!(estimator.kf_vertical_velocity(), None, "no filter on the pad");
    assert!(
        (estimator.launch_pad_altitude_asl() - pad).abs() < 5.0,
        "pad altitude={} expected ~{}",
        estimator.launch_pad_altitude_asl(),
        pad
    );

    // Boost until ignition detection drops the estimator into the lockout.
    let mut altitude_asl = pad;
    let mut velocity = 0.0f32;
    while !matches!(estimator.state(), RocketState::MachLockout { .. }) {
        velocity += 80.0 * DT;
        altitude_asl += velocity * DT;
        estimator.update(clock.tick(), sf(80.0 + 9.81), altitude_asl + noise.next());
        assert!(
            altitude_asl - pad < 1000.0,
            "lockout never engaged by {}m agl",
            altitude_asl - pad
        );
    }

    // The frozen filter's contents are now drifting further out of date every
    // sample; nothing may read them, however plausible they still look.
    for _ in 0..SAMPLES_PER_S {
        velocity += 80.0 * DT;
        altitude_asl += velocity * DT;
        estimator.update(clock.tick(), sf(80.0 + 9.81), altitude_asl + noise.next());
        assert!(matches!(estimator.state(), RocketState::MachLockout { .. }));
        assert_eq!(estimator.kf_altitude_asl(), None);
        assert_eq!(estimator.kf_vertical_velocity(), None);
    }
}

/// Full replay of the real Void Lake flight log (raw MS5607 pressure,
/// including the redundant computer's ejection-blast transient at apogee, the
/// boost static-port overshoot, and the 1 Hz sensor stalls — the stalls are
/// bridged by interpolating onto the estimator's fixed 416 Hz grid, matching
/// the stall-free stream the machine is designed for).
///
/// Subsonic flight: no Mach lockout. Dual profile matching the flight.
#[test]
fn void_lake_flight_replay() {
    let mut clock = SampleClock::new();
    use icao_isa::calculate_isa_altitude;
    use icao_units::si::Pascals;

    crate::tests::init_logger();

    #[derive(serde::Deserialize)]
    struct Row {
        timestamp_us: u64,
        pressure: f32,
        baro_valid: bool,
        acc_x: f32,
        acc_y: f32,
        acc_z: f32,
        imu_valid: bool,
    }

    let mut reader = csv::Reader::from_path("./test_data/void_lake_flight.csv").unwrap();
    // Accelerometer alongside the pressure, because ignition detection is
    // the accelerometer's job now — a baro-only replay of this flight would
    // never leave the pad. `None` on the rows the recorder marked invalid,
    // which is the same thing the firmware puts on the wire.
    let samples: Vec<(u64, f32, Option<Vector3<f32>>)> = reader
        .deserialize::<Row>()
        .map(|r| r.unwrap())
        .filter(|r| r.baro_valid && r.pressure > 10_000.0)
        .map(|r| {
            (
                r.timestamp_us,
                calculate_isa_altitude(Pascals(r.pressure as f64)).0 as f32,
                r.imu_valid
                    .then(|| Vector3::new(r.acc_x, r.acc_y, r.acc_z)),
            )
        })
        .collect();
    assert!(samples.len() > 50_000, "unexpectedly few samples");

    // Interpolate onto the fixed-rate grid the estimator assumes. Altitude
    // is interpolated; the accelerometer is taken from the bracketing sample
    // as-is, since a threshold has no use for a fractional one.
    let t0 = samples[0].0;
    let t_end = samples.last().unwrap().0;
    let dt_us = (DT * 1_000_000.0) as u64;
    let mut grid = Vec::with_capacity(((t_end - t0) / dt_us + 1) as usize);
    let mut j = 0usize;
    let mut t = t0;
    while t <= t_end {
        while j + 1 < samples.len() && samples[j + 1].0 <= t {
            j += 1;
        }
        let alt = if j + 1 < samples.len() {
            let (ta, aa, _) = samples[j];
            let (tb, ab, _) = samples[j + 1];
            let frac = (t - ta) as f32 / (tb - ta).max(1) as f32;
            aa + (ab - aa) * frac
        } else {
            samples[j].1
        };
        grid.push((alt, samples[j].2));
        t += dt_us;
    }

    // Flight references derived from the data itself.
    let pad_ref: f32 =
        grid[..5 * SAMPLES_PER_S].iter().map(|(a, _)| *a).sum::<f32>()
            / (5 * SAMPLES_PER_S) as f32;
    let (apogee_i, apogee_alt) = grid
        .iter()
        .enumerate()
        .fold((0, f32::MIN), |(bi, ba), (i, (a, _))| {
            if *a > ba { (i, *a) } else { (bi, ba) }
        });
    let liftoff_i = grid
        .iter()
        .position(|(a, _)| *a > pad_ref + 15.0)
        .expect("no liftoff in data");

    // Void Lake's own dual-deploy numbers, on the subsonic base.
    let mut estimator = RocketStateEstimator::new(
        FlightProfile {
            deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 500.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: 457.2,
                main_chute_delay_us: 0,
            },
            ..subsonic_profile()
        },
        IGNITION_ACC_THRESHOLD,
    );

    let mut ignition_i = None;
    let mut drogue = None;
    let mut main = None;
    for (i, &(alt, acc)) in grid.iter().enumerate() {
        let (pyro, _gate) = estimator.update(clock.tick(), acc, alt);
        assert!(!matches!(
            estimator.state(),
            RocketState::MachLockout { .. }
        ));
        if ignition_i.is_none() && !matches!(estimator.state(), RocketState::OnPad) {
            ignition_i = Some(i);
        }
        match pyro {
            Some(PyroSelect::PyroDrogue) => {
                assert!(drogue.is_none());
                drogue = Some((i, alt));
            }
            Some(PyroSelect::PyroMain) => {
                assert!(main.is_none());
                main = Some((i, alt));
            }
            None => {}
        }
    }
    // The log ends shortly after touchdown; keep feeding the final altitude
    // so the 5 s landed persistence can complete.
    let (last_alt, last_acc) = *grid.last().unwrap();
    let mut noise = NoiseGen::new(0.5);
    for _ in 0..(15 * SAMPLES_PER_S) {
        estimator.update(clock.tick(), last_acc, last_alt + noise.next());
    }

    let ignition_i = ignition_i.expect("ignition never detected");
    assert!(
        ignition_i > liftoff_i && ignition_i < liftoff_i + 3 * SAMPLES_PER_S,
        "ignition detected at grid {} vs liftoff {}",
        ignition_i,
        liftoff_i
    );

    let (drogue_i, drogue_alt) = drogue.expect("drogue never fired");
    assert!(
        drogue_i >= apogee_i.saturating_sub(2 * SAMPLES_PER_S)
            && drogue_i < apogee_i + 8 * SAMPLES_PER_S,
        "drogue at grid {} vs apogee {}",
        drogue_i,
        apogee_i
    );
    assert!(
        (drogue_alt - apogee_alt).abs() < 100.0,
        "drogue alt={} apogee alt={}",
        drogue_alt,
        apogee_alt
    );

    let (main_i, main_alt) = main.expect("main never fired");
    assert!(main_i > drogue_i);
    assert!(
        (main_alt - (pad_ref + 457.2)).abs() < 100.0,
        "main fired at {} m ASL, expected ~{}",
        main_alt,
        pad_ref + 457.2
    );

    assert!(
        matches!(estimator.state(), RocketState::Landed),
        "final state {:?}",
        estimator.state()
    );
}

#[test]
fn innovation_gate_rejects_pyro_transient() {
    let mut kf = BaroAltitudeKF::new(200.0);
    let mut noise = NoiseGen::new(0.5);
    for _ in 0..(2 * SAMPLES_PER_S) {
        kf.predict();
        assert_eq!(kf.update(200.0 + noise.next()), BaroGateOutcome::Accepted);
    }

    // ejection-charge overpressure: ~60 ms of readings up to 1400 m low
    for _ in 0..25 {
        kf.predict();
        assert_eq!(
            kf.update(-1200.0),
            BaroGateOutcome::Rejected,
            "transient sample must be rejected"
        );
    }
    assert!((kf.altitude_asl() - 200.0).abs() < 1.0, "altitude held through transient");
    assert!(kf.vertical_velocity().abs() < 1.0, "velocity held through transient");

    // clean data is accepted again immediately, no recovery period
    kf.predict();
    assert_eq!(kf.update(200.0), BaroGateOutcome::Accepted);
}

#[test]
fn innovation_gate_force_accepts_persistent_offset() {
    let mut kf = BaroAltitudeKF::new(200.0);
    for _ in 0..SAMPLES_PER_S {
        kf.predict();
        kf.update(200.0);
    }

    // a persistent 800 m offset is a diverged filter, not a transient: after
    // 1 s of rejections the gate must give up and snap to the measurement
    let mut accepted = 0u32;
    let mut resyncs = 0u32;
    for _ in 0..(2 * SAMPLES_PER_S) {
        kf.predict();
        match kf.update(1000.0) {
            BaroGateOutcome::Rejected => {}
            BaroGateOutcome::Resynced => {
                resyncs += 1;
                accepted += 1;
            }
            BaroGateOutcome::Accepted => accepted += 1,
        }
    }
    assert!(accepted > 0, "gate never re-accepted");
    // Reported on the single sample it happens on, not left for a poller.
    assert_eq!(resyncs, 1, "the snap is reported exactly once");
    assert!(
        (kf.altitude_asl() - 1000.0).abs() < 5.0,
        "filter did not re-converge, altitude={}",
        kf.altitude_asl()
    );
}

// ===========================================================================
// Sample-rate independence
// ===========================================================================

/// A closed-form flight, evaluated at wall-clock time, so every sample rate
/// below flies the exact same airframe rather than its own integration of
/// one. Pad hold, constant-thrust boost, ballistic coast, then a capped
/// descent to the ground.
struct AnalyticFlight {
    pad_asl: f32,
}

impl AnalyticFlight {
    const PAD_HOLD_S: f32 = 12.0;
    const BURN_S: f32 = 3.0;
    const BURN_ACCEL: f32 = 90.0;
    const G: f32 = 9.81;
    const TERMINAL_V: f32 = 60.0;

    /// Altitude ASL at wall-clock time `t` (s from the first sample).
    fn altitude_asl(&self, t: f32) -> f32 {
        let tf = t - Self::PAD_HOLD_S;
        if tf <= 0.0 {
            return self.pad_asl;
        }
        if tf < Self::BURN_S {
            return self.pad_asl + 0.5 * Self::BURN_ACCEL * tf * tf;
        }
        let v0 = Self::BURN_ACCEL * Self::BURN_S;
        let h0 = 0.5 * Self::BURN_ACCEL * Self::BURN_S * Self::BURN_S;
        let tc = tf - Self::BURN_S;

        // Free flight until the descent reaches terminal velocity, then a
        // straight line down at that speed.
        let t_term = (v0 + Self::TERMINAL_V) / Self::G;
        let h = if tc <= t_term {
            h0 + v0 * tc - 0.5 * Self::G * tc * tc
        } else {
            let h_term = h0 + v0 * t_term - 0.5 * Self::G * t_term * t_term;
            h_term - Self::TERMINAL_V * (tc - t_term)
        };
        self.pad_asl + h.max(0.0)
    }

    /// Vertical velocity (m/s) at wall-clock time `t`, differentiated from
    /// [`Self::altitude_asl`] rather than written out again, so the two can
    /// never disagree about the flight. `h` is well inside the shortest
    /// phase (the 3 s burn).
    fn vertical_velocity(&self, t: f32) -> f32 {
        const H: f32 = 0.001;
        (self.altitude_asl(t + H) - self.altitude_asl(t - H)) / (2.0 * H)
    }

    /// Specific force along the airframe axis at wall-clock time `t`
    /// (m/s^2) — what an accelerometer measures, so gravity is absent in
    /// free flight and present whenever something is holding the airframe
    /// up. Matches the convention `hil::osiris` replays.
    fn specific_force(&self, t: f32) -> f32 {
        let tf = t - Self::PAD_HOLD_S;
        if tf <= 0.0 {
            // on the rail, the pad carries the weight
            Self::G
        } else if tf < Self::BURN_S {
            // thrust also has to lift the weight
            Self::BURN_ACCEL + Self::G
        } else {
            let v0 = Self::BURN_ACCEL * Self::BURN_S;
            let tc = tf - Self::BURN_S;
            if tc <= (v0 + Self::TERMINAL_V) / Self::G {
                0.0 // ballistic: free fall senses nothing
            } else {
                Self::G // descending at terminal velocity, aerodynamically supported
            }
        }
    }

    /// Wall-clock time the airframe reaches the ground.
    fn touchdown_s(&self) -> f32 {
        let mut t = Self::PAD_HOLD_S + Self::BURN_S;
        while self.altitude_asl(t) > self.pad_asl + 0.01 {
            t += 0.01;
        }
        t
    }
}

/// Every event the timers below are measured between, in wall-clock seconds
/// from the first sample. `None` means it never happened.
#[derive(Debug, Default)]
struct Events {
    ignition_detected: Option<f32>,
    lockout_exit: Option<f32>,
    drogue_delay_start: Option<f32>,
    drogue_fire: Option<f32>,
    main_delay_start: Option<f32>,
    main_fire: Option<f32>,
    landed: Option<f32>,
}

/// Fly `flight` at `hz` and record when each event happened.
fn fly_at_rate(hz: f64, profile: FlightProfile, flight: &AnalyticFlight) -> Events {
    let mut estimator = RocketStateEstimator::new(profile, IGNITION_ACC_THRESHOLD);
    let mut noise = NoiseGen::new(0.5);
    let mut ev = Events::default();

    let dt_ns = (1e9 / hz) as u64;
    let end_s = flight.touchdown_s() + 20.0;
    let mut ns = 0u64;

    let set = |slot: &mut Option<f32>, t: f32| {
        if slot.is_none() {
            *slot = Some(t);
        }
    };

    while (ns as f32) * 1e-9 < end_s {
        let t = (ns as f64 * 1e-9) as f32;
        // Round rather than truncate to microseconds: a real clock reports
        // whole microseconds with unbiased rounding, and truncating here
        // would shave half a microsecond off every measured dt — 2.3 ms
        // over a 12 s lockout, which is the test injecting a bias rather
        // than measuring one.
        let (pyro, _gate) = estimator.update(
            (ns + 500) / 1000,
            sf(flight.specific_force(t)),
            flight.altitude_asl(t) + noise.next(),
        );

        match estimator.state() {
            RocketState::OnPad => {}
            RocketState::MachLockout { .. } => set(&mut ev.ignition_detected, t),
            RocketState::Ascent { .. } => {
                set(&mut ev.ignition_detected, t);
                set(&mut ev.lockout_exit, t);
            }
            RocketState::DrogueChute { deployed: false, .. } => {
                set(&mut ev.drogue_delay_start, t)
            }
            RocketState::MainChute { deployed: false, .. } => set(&mut ev.main_delay_start, t),
            RocketState::Landed => set(&mut ev.landed, t),
            _ => {}
        }
        match pyro {
            Some(PyroSelect::PyroDrogue) => set(&mut ev.drogue_fire, t),
            Some(PyroSelect::PyroMain) => set(&mut ev.main_fire, t),
            None => {}
        }
        ns += dt_ns;
    }
    ev
}

/// The point of taking a timestamp: every DURATION the deployment state
/// machine measures must come out in honest seconds, whatever rate the
/// sensor actually runs at.
///
/// This is not hypothetical. The LSM6DSM's "416 Hz" ODR measures 427.02 Hz
/// on the VLF5 board (`scripts/imu_bench_stats.py`), so when these timers
/// were counts of samples every one of them expired 2.65% early — a 26 s
/// Mach lockout ran 25.33 s, a 1 s drogue delay ran 0.977 s. The sweep
/// below spans a wider band than any one part will drift, and the old
/// sample-counted machine would fail it by whole seconds on the lockout.
///
/// Deliberately NOT asserted: that the events themselves land at the same
/// wall-clock time. The KF is sample-clocked on purpose (see
/// [`SAMPLES_PER_S`]) — it advances one fixed `DT` per sample regardless —
/// so feeding it faster genuinely changes its bandwidth in real time, and
/// with it how quickly it notices ignition and apogee. That is the trade
/// the split was chosen for: the filter that fires the pyros cannot be
/// surprised by a clock, and the timers that have to mean seconds read one.
#[test]
fn timers_are_independent_of_the_sample_rate() {
    let flight = AnalyticFlight { pad_asl: 200.0 };
    // Every duration below is checked against wall clock at four sample
    // rates, so all four of them stay here where the assertion can be read
    // against them: a 12 s lockout, a 1 s drogue delay, a 500 ms main.
    let profile = FlightProfile {
        mach_lockout_duration_us: 12_000_000.into(),
        deployment: DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: 500.0,
            drogue_chute_delay_us: 1_000_000,
            main_chute_altitude_agl: 500.0,
            main_chute_delay_us: 500_000,
        },
        ..subsonic_profile()
    };

    // A band comfortably wider than real part-to-part ODR variation.
    let rates = [380.0f64, 416.0, 427.02, 480.0];
    let mut landed = Vec::new();

    for hz in rates {
        let ev = fly_at_rate(hz, profile.clone(), &flight);
        // A timer is checked at the top of each sample and fires on the
        // first one where it has expired, so it always lands a sample or
        // two LATE and never meaningfully early. Assert that directly:
        // erring late costs milliseconds, erring early fires a pyro before
        // its time.
        //
        // "Meaningfully" is one sample interval. Counting a 12 s deadline
        // down in ~2.4 ms f32 steps accumulates rounding across the five
        // thousand subtractions it takes — measured worst case -565 us at
        // 427 Hz, which is the arithmetic rather than the timer, and four
        // orders of magnitude below the 690 ms that counting samples cost
        // at the same rate. A real regression is not subtle: a
        // sample-counted 12 s lockout at 427 Hz expires 300 samples early.
        let sample_s = 1.0 / hz as f32;
        let late = |measured: f32, nominal: f32| {
            let over = measured - nominal;
            over > -sample_s && over < 3.0 * sample_s
        };

        let det = ev.ignition_detected.expect("ignition never detected");
        let exit = ev.lockout_exit.expect("lockout never exited");
        let dd = ev.drogue_delay_start.expect("descent never detected");
        let df = ev.drogue_fire.expect("drogue never fired");
        let md = ev.main_delay_start.expect("main altitude never reached");
        let mf = ev.main_fire.expect("main never fired");
        let ld = ev.landed.expect("never landed");

        eprintln!(
            "{hz:6.2} Hz: lockout {:+.7}s, drogue delay {:+.7}s, main delay {:+.7}s \
             OVER nominal (ignition detected at {det:.3}s, landed at {ld:.2}s)",
            exit - det - 12.0,
            df - dd - 1.0,
            mf - md - 0.5,
        );

        assert!(
            late(exit - det, 12.0),
            "{hz} Hz: 12 s Mach lockout measured {:.4}s",
            exit - det
        );
        assert!(
            late(df - dd, 1.0),
            "{hz} Hz: 1 s drogue delay measured {:.4}s",
            df - dd
        );
        assert!(
            late(mf - md, 0.5),
            "{hz} Hz: 0.5 s main delay measured {:.4}s",
            mf - md
        );
        landed.push(ld);
    }

    // The landed latch is a pure 5 s timer, but what it counts from is the
    // KF's velocity settling under the threshold — and the KF is
    // sample-clocked, so this is where the split's cost shows up.
    //
    // Touchdown happens at the same wall-clock instant at every rate, but
    // the filter needs a fixed number of SAMPLES to bleed 60 m/s of
    // descent down to the 2 m/s threshold, so it gets there sooner in real
    // time the faster it is fed. Measured spread across 380-480 Hz: 2.3 s,
    // all of it after the rocket is already on the ground and both pyros
    // have fired. That is the accepted price of a filter that cannot be
    // surprised by a clock; nothing latency-sensitive reads it.
    let spread = landed.iter().cloned().fold(f32::MIN, f32::max)
        - landed.iter().cloned().fold(f32::MAX, f32::min);
    eprintln!("landing detection spread across rates: {spread:.2}s (KF is sample-clocked)");
    assert!(
        spread < 3.0,
        "landing detection spread {spread:.3}s across rates — the KF's \
         sample-clocked settling should account for ~2.3s, not this"
    );
}

/// A gap in the sample stream is real elapsed time, and the timers count
/// it. This is the case the sample-counted machine could not get right: a
/// delay only advanced when a sample arrived, so half a second of lost
/// samples stretched a 1 s drogue delay to 1.5 s of wall clock, and the
/// flight log said nothing about it.
///
/// There is deliberately no ceiling on the per-sample dt. One would put
/// exactly that error back — capped instead of unbounded, but still
/// under-counting the delay by whatever the stream lost.
#[test]
fn a_sensor_stall_does_not_stretch_a_pyro_delay() {
    const STALL_S: f32 = 0.4;

    let flight = AnalyticFlight { pad_asl: 200.0 };
    // The 1 s drogue delay is the quantity under test — it is what the
    // 0.4 s stall is dropped into and what the assertion measures.
    let profile = FlightProfile {
        deployment: DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: 500.0,
            drogue_chute_delay_us: 1_000_000,
            main_chute_altitude_agl: 500.0,
            main_chute_delay_us: 0,
        },
        ..subsonic_profile()
    };

    let mut estimator = RocketStateEstimator::new(profile, IGNITION_ACC_THRESHOLD);
    let mut noise = NoiseGen::new(0.5);
    let mut clock = SampleClock::new();
    let mut delay_start: Option<f32> = None;
    let mut fire: Option<f32> = None;
    let mut stalled_until: Option<f32> = None;

    let end = ((flight.touchdown_s() + 20.0) * SAMPLES_PER_S as f32) as usize;
    for _ in 0..end {
        // The clock ticks whether or not the sample is delivered — that is
        // what a dropped sample looks like from in here.
        let t_us = clock.tick();
        let t = t_us as f32 * 1e-6;

        // Drop every sample for STALL_S once the drogue delay is running,
        // so the whole gap lands inside the delay being measured.
        if let Some(until) = stalled_until {
            if t < until {
                continue;
            }
        }

        let (pyro, _gate) = estimator.update(
            t_us,
            sf(flight.specific_force(t)),
            flight.altitude_asl(t) + noise.next(),
        );

        if matches!(
            estimator.state(),
            RocketState::DrogueChute { deployed: false, .. }
        ) && delay_start.is_none()
        {
            delay_start = Some(t);
            stalled_until = Some(t + STALL_S);
        }
        if matches!(pyro, Some(PyroSelect::PyroDrogue)) && fire.is_none() {
            fire = Some(t);
        }
    }

    let start = delay_start.expect("descent never detected");
    let fired = fire.expect("drogue never fired across the stall");
    let measured = fired - start;
    eprintln!("1 s drogue delay across a {STALL_S}s stall measured {measured:.4}s");
    assert!(
        (measured - 1.0).abs() < 0.01,
        "1 s drogue delay measured {measured:.4}s across a {STALL_S}s sensor stall"
    );
}

/// Ignition comes from the accelerometer and from nothing else.
///
/// Two halves, and the second is as much the point as the first: the motor
/// must be found promptly, and with no accelerometer the estimator must sit
/// on the pad forever rather than invent a launch from the barometer. That
/// second half is a real failure mode, deliberately chosen — an IMU that
/// reads successfully but reports low means no drogue and no main — so it
/// is written down here rather than left to be discovered.
#[test]
fn ignition_comes_from_the_accelerometer_alone() {
    let flight = AnalyticFlight { pad_asl: 200.0 };

    let detect = |feed_imu: bool| -> Option<f32> {
        // What is measured is the instant the state leaves `OnPad`, which
        // no chute number touches.
        let mut estimator = RocketStateEstimator::new(subsonic_profile(), IGNITION_ACC_THRESHOLD);
        let mut noise = NoiseGen::new(0.5);
        let mut clock = SampleClock::new();
        let end = ((flight.touchdown_s() + 20.0) * SAMPLES_PER_S as f32) as usize;
        for _ in 0..end {
            let t_us = clock.tick();
            let t = t_us as f32 * 1e-6;
            let acc = if feed_imu {
                sf(flight.specific_force(t))
            } else {
                None
            };
            estimator.update(t_us, acc, flight.altitude_asl(t) + noise.next());
            if !matches!(estimator.state(), RocketState::OnPad) {
                return Some(t);
            }
        }
        None
    };

    let motor_lit = AnalyticFlight::PAD_HOLD_S;
    let detected = detect(true).expect("ignition never detected with a healthy IMU");
    eprintln!("ignition detected {:+.3}s after the motor lit", detected - motor_lit);

    // After the motor lit, and quickly — the 10 Hz low pass plus the 0.1 s
    // sustain is the whole of the delay.
    assert!(
        detected > motor_lit,
        "ignition latched {:.3}s BEFORE the motor lit",
        motor_lit - detected
    );
    assert!(
        detected - motor_lit < 0.3,
        "ignition took {:.3}s to latch on a 10 g motor",
        detected - motor_lit
    );

    // The whole flight, barometrically perfect, with no accelerometer: the
    // barometer is not consulted about ignition and nothing happens.
    assert_eq!(
        detect(false),
        None,
        "ignition was detected without an accelerometer — the barometer is \
         not supposed to have a vote"
    );
}

/// A knock on the rail is not a launch. The 10 Hz low pass lets a short
/// transient through at surprising amplitude — a 40 ms burst of 12 g reaches
/// well past a 4 g threshold — so what actually rejects it is the sustain:
/// the channel has to STAY up, and a knock does not.
#[test]
fn a_knock_on_the_pad_does_not_latch_ignition() {
    // Restated rather than taken from `IGNITION_ACC_THRESHOLD`: the doc
    // above is an argument about 12 g knocks against a 4 g threshold, and
    // the number it argues from belongs next to it.
    const THRESHOLD: f32 = 4.0 * 9.81;

    let flight = AnalyticFlight { pad_asl: 200.0 };
    let mut estimator = RocketStateEstimator::new(subsonic_profile(), THRESHOLD);
    let mut noise = NoiseGen::new(0.5);
    let mut clock = SampleClock::new();
    let mut peak_lp = 0.0f32;
    let mut lp: Option<Vector3<f32>> = None;

    // 10 s on the pad, with a 40 ms 12 g knock every second.
    for _ in 0..(10 * SAMPLES_PER_S) {
        let t_us = clock.tick();
        let t = t_us as f32 * 1e-6;
        let knocking = (t % 1.0) < 0.04 && t > 1.0;
        let acc = if knocking {
            Vector3::new(0.0, 0.0, 12.0 * 9.81)
        } else {
            Vector3::new(0.0, 0.0, 9.81)
        };
        estimator.update(t_us, Some(acc), flight.altitude_asl(t) + noise.next());

        // Mirror of the estimator's low pass, to show the transient really
        // does cross the threshold and that the sustain is what holds.
        // Fed at the nominal tick, so one constant alpha; 0.0159 s is
        // `ignition_detector::LP_TAU_S`, shared with the airbrakes half.
        let alpha = (DT / 0.0159f32).min(1.0);
        let v = match lp {
            Some(prev) => prev + alpha * (acc - prev),
            None => acc,
        };
        lp = Some(v);
        peak_lp = peak_lp.max(v.magnitude());

        assert!(
            matches!(estimator.state(), RocketState::OnPad),
            "a knock at t={t:.3}s latched ignition"
        );
    }
    eprintln!(
        "knock test: worst low-passed |acc| reached {:.1} m/s^2 = {:.1} g \
         (threshold 4.0 g), and nothing latched",
        peak_lp,
        peak_lp / 9.81
    );
    assert!(
        peak_lp > 4.0 * 9.81,
        "the knock never crossed the threshold ({:.1} m/s^2) — this test proves nothing",
        peak_lp
    );
}

// ===========================================================================
// Birth of the in-flight filter
// ===========================================================================

/// One bad barometer reading landing on the sample that ends the Mach
/// lockout must not decide the flight.
///
/// That sample used to be the entire birth: the filter was seeded from it
/// and `peak_altitude_asl` was seeded from it, with nothing in between —
/// the old filter had been dropped at ignition, so its innovation gate was
/// not there to reject anything, and the pad gate does not run past the
/// pad. A high reading therefore set a peak the rocket could never reach,
/// and the first honest sample afterwards read as kilometres of descent.
///
/// Measured on the Osiris O3400 simulation (`tests::osiris_sim`, the flown
/// `FLIGHT_CONFIG` with its 26 s lockout), one bad sample on the exit
/// sample only:
///
/// | reading injected | drogue | true vv there |
/// |---|---|---|
/// | none (nominal) | 43.60 s | -25.5 m/s |
/// | 30000 m (SPI read decoding to pressure ~0) | **28.67 s** | **+109.7 m/s** |
/// | 12854 m (plain factor-of-2 pressure error) | **28.67 s** | **+109.7 m/s** |
/// | 3288 m (factor of 2 the other way) | 43.60 s | -25.5 m/s |
///
/// Firing the drogue at +109.7 m/s of ascent is a structural failure, and
/// it took no exotic fault to get there — half the pressure was enough.
/// With the birth taken from a median of the last `BARO_RING_SPAN_S`
/// instead, all four cases fire at 43.60 s, unchanged from nominal.
///
/// This test flies the same shape on the closed-form flight above so it
/// owns its own numbers: a 12 s lockout that ends at +180 m/s of climb, and
/// a 30 km reading placed on exactly the sample the filter is born from.
/// The load-bearing assertion is the sign of the velocity at the fire — a
/// drogue that comes out while the rocket is still going up is the failure
/// this guards, and no tolerance on a timestamp expresses that.
#[test]
fn a_garbage_sample_at_lockout_exit_does_not_fire_the_drogue() {
    const LOCKOUT_S: f32 = 12.0;
    /// What a bad SPI read looks like once it has been through the pressure
    /// conversion: pressure ~0 is roughly a 30 km altitude.
    const GARBAGE_ASL: f32 = 30_000.0;

    let flight = AnalyticFlight { pad_asl: 200.0 };
    // The lockout duration decides which sample the filter is born from —
    // the sample the garbage is injected on — and the 1 s delay is what
    // separates the fire from that birth, so both stay here.
    let profile = FlightProfile {
        mach_lockout_duration_us: Some((LOCKOUT_S * 1e6) as u32),
        deployment: DeploymentProfile::Single {
            minimum_deployment_altitude_agl: 500.0,
            delay_us: 1_000_000,
        },
        ..subsonic_profile()
    };

    // Fly once, optionally replacing the reading on sample `inject_at`.
    // The noise is drawn on every sample whether or not it is used, so the
    // clean and injected flights see the identical noise sequence and any
    // difference between them is the injected sample and nothing else.
    //
    // Returns the index of the sample the lockout ended on — which is the
    // sample the filter is born from — and the drogue fire as
    // (wall-clock time, TRUE vertical velocity there).
    let fly = |inject_at: Option<usize>| -> (Option<(usize, f32)>, Option<(f32, f32)>) {
        let mut estimator = RocketStateEstimator::new(profile.clone(), IGNITION_ACC_THRESHOLD);
        let mut noise = NoiseGen::new(0.5);
        let mut clock = SampleClock::new();
        let mut in_lockout = false;
        let mut exit = None;
        let mut drogue = None;

        let end = ((flight.touchdown_s() + 20.0) * SAMPLES_PER_S as f32) as usize;
        for i in 0..end {
            let t_us = clock.tick();
            let t = t_us as f32 * 1e-6;
            let honest = flight.altitude_asl(t) + noise.next();
            let measured = if inject_at == Some(i) {
                GARBAGE_ASL
            } else {
                honest
            };
            let (pyro, _gate) = estimator.update(t_us, sf(flight.specific_force(t)), measured);

            match estimator.state() {
                RocketState::MachLockout { .. } => in_lockout = true,
                _ if in_lockout && exit.is_none() => exit = Some((i, t)),
                _ => {}
            }
            if matches!(pyro, Some(PyroSelect::PyroDrogue)) && drogue.is_none() {
                drogue = Some((t, flight.vertical_velocity(t)));
            }
        }
        (exit, drogue)
    };

    let (exit, clean) = fly(None);
    let (exit_i, exit_t) = exit.expect("lockout never exited");
    let (clean_t, clean_vv) = clean.expect("no drogue on the clean flight");

    // The injected sample has to land where it does damage, so check that
    // the birth really is mid-ascent before trusting what follows.
    let exit_vv = flight.vertical_velocity(exit_t);
    assert!(
        exit_vv > 100.0,
        "the lockout ends at {exit_vv:+.1} m/s — this test only means \
         something if the filter is born while the rocket is climbing hard"
    );

    let (_, dirty) = fly(Some(exit_i));
    let (dirty_t, dirty_vv) = dirty.expect(
        "no drogue fired at all after one garbage sample: the filter was born \
         30 km up and never re-acquired — a ballistic descent",
    );

    eprintln!(
        "garbage birth: lockout ends on sample {exit_i} at t={exit_t:.2}s \
         ({exit_vv:+.1} m/s) | drogue clean {clean_t:.2}s ({clean_vv:+.1} m/s), \
         with one {GARBAGE_ASL:.0} m sample on that exact sample {dirty_t:.2}s \
         ({dirty_vv:+.1} m/s)"
    );

    assert!(
        dirty_vv < 0.0,
        "the drogue fired at {dirty_vv:+.1} m/s — still ascending — {:.2}s \
         before the clean flight's apogee deployment",
        clean_t - dirty_t
    );
    assert!(
        (dirty_t - clean_t).abs() < 0.5,
        "one garbage sample moved the drogue by {:+.2}s ({dirty_t:.2}s vs \
         {clean_t:.2}s): the birth is still being decided by a single reading",
        dirty_t - clean_t
    );
}
