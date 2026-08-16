use super::*;

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
    let mut noise = NoiseGen::new(0.5);
    let mut drogue = None;
    let mut main = None;
    let mut sample_i = 0usize;

    let mut feed = |estimator: &mut RocketStateEstimator, altitude_asl: f32| {
        let pyro = estimator.update(altitude_asl + noise.next());
        // Pyros only fire in chute stages, where the KF is live — reading
        // the raw KF altitude here is safe.
        let kf_agl = estimator.kf_altitude_asl() - estimator.launch_pad_altitude_asl();
        match pyro {
            Some(PyroSelect::PyroDrogue) => {
                assert!(drogue.is_none(), "drogue fired more than once");
                drogue = Some((sample_i, kf_agl));
            }
            Some(PyroSelect::PyroMain) => {
                assert!(main.is_none(), "main fired more than once");
                main = Some((sample_i, kf_agl));
            }
            None => {}
        }
        sample_i += 1;
    };

    // 30 s sitting on the pad
    for _ in 0..(30 * SAMPLES_PER_S) {
        feed(estimator, pad_altitude_asl);
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
        feed(estimator, altitude_asl);
    }

    // 30 s sitting on the ground
    for _ in 0..(30 * SAMPLES_PER_S) {
        feed(estimator, pad_altitude_asl);
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
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None,
        deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 500.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: main_agl,
                main_chute_delay_us: 0,
        },
    });

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
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None,
        deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 500.0,
                delay_us: 0,
        },
    });

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
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None,
        deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 5000.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: 457.2,
                main_chute_delay_us: 0,
        },
    });

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
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None,
        deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 300.0,
                delay_us: 0,
        },
    });
    let mut noise = NoiseGen::new(0.5);

    // 2 minutes armed on the pad
    for _ in 0..(120 * SAMPLES_PER_S) {
        estimator.update(200.0 + noise.next());
    }
    assert!(matches!(estimator.state(), RocketState::OnPad));
}

/// Redundant-computer ejection blast during coast (60 ms of readings ~1400 m
/// low, injected 3 s before apogee): the innovation gate must reject every
/// sample and apogee detection must not fire early.
#[test]
fn blast_transient_during_coast_does_not_deploy_early() {
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None,
        deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 300.0,
                delay_us: 0,
        },
    });
    let mut noise = NoiseGen::new(0.5);
    let pad = 200.0f32;

    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(pad + noise.next());
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
        if let Some(PyroSelect::PyroDrogue) = estimator.update(measured) {
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
    // Sim profile: 6.5 s burn at 100 m/s^2 -> 650 m/s (~Mach 1.9),
    // gravity-only coast, apogee ~72.8 s. Ignition detection completes ~1.5 s
    // into the burn; back below Mach 0.75 at ~43 s. Lockout = 48 s from
    // detection (~margin), ending ~23 s before apogee.
    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: Some(48_000_000),
        deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 1000.0,
                delay_us: 0,
        },
    });
    let mut noise = NoiseGen::new(0.5);
    let pad = 200.0f32;

    for _ in 0..(30 * SAMPLES_PER_S) {
        estimator.update(pad + noise.next());
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
        let pyro = estimator.update(measured);
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
        estimator.update(pad + noise.next());
    }
    assert!(matches!(estimator.state(), RocketState::Landed));
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
    use icao_isa::calculate_isa_altitude;
    use icao_units::si::Pascals;

    crate::tests::init_logger();

    #[derive(serde::Deserialize)]
    struct Row {
        timestamp_us: u64,
        pressure: f32,
        baro_valid: bool,
    }

    let mut reader = csv::Reader::from_path("./test_data/void_lake_flight.csv").unwrap();
    let samples: Vec<(u64, f32)> = reader
        .deserialize::<Row>()
        .map(|r| r.unwrap())
        .filter(|r| r.baro_valid && r.pressure > 10_000.0)
        .map(|r| {
            (
                r.timestamp_us,
                calculate_isa_altitude(Pascals(r.pressure as f64)).0 as f32,
            )
        })
        .collect();
    assert!(samples.len() > 50_000, "unexpectedly few samples");

    // Interpolate onto the fixed-rate grid the estimator assumes.
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
            let (ta, aa) = samples[j];
            let (tb, ab) = samples[j + 1];
            let frac = (t - ta) as f32 / (tb - ta).max(1) as f32;
            aa + (ab - aa) * frac
        } else {
            samples[j].1
        };
        grid.push(alt);
        t += dt_us;
    }

    // Flight references derived from the data itself.
    let pad_ref: f32 =
        grid[..5 * SAMPLES_PER_S].iter().sum::<f32>() / (5 * SAMPLES_PER_S) as f32;
    let (apogee_i, apogee_alt) = grid
        .iter()
        .enumerate()
        .fold((0, f32::MIN), |(bi, ba), (i, &a)| {
            if a > ba { (i, a) } else { (bi, ba) }
        });
    let liftoff_i = grid
        .iter()
        .position(|&a| a > pad_ref + 15.0)
        .expect("no liftoff in data");

    let mut estimator = RocketStateEstimator::new(FlightProfile {
        mach_lockout_duration_us: None, // subsonic flight
        deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 500.0,
                drogue_chute_delay_us: 0,
                main_chute_altitude_agl: 457.2,
                main_chute_delay_us: 0,
        },
    });

    let mut ignition_i = None;
    let mut drogue = None;
    let mut main = None;
    for (i, &alt) in grid.iter().enumerate() {
        let pyro = estimator.update(alt);
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
    let last_alt = *grid.last().unwrap();
    let mut noise = NoiseGen::new(0.5);
    for _ in 0..(15 * SAMPLES_PER_S) {
        estimator.update(last_alt + noise.next());
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
