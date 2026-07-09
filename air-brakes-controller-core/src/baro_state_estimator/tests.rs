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
        match pyro {
            Some(PyroSelect::PyroDrogue) => {
                assert!(drogue.is_none(), "drogue fired more than once");
                drogue = Some((sample_i, estimator.altitude_agl()));
            }
            Some(PyroSelect::PyroMain) => {
                assert!(main.is_none(), "main fired more than once");
                main = Some((sample_i, estimator.altitude_agl()));
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
    let mut altitude = pad_altitude_asl;
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
        altitude += velocity * DT;
        t += DT;

        if altitude <= pad_altitude_asl {
            altitude = pad_altitude_asl;
            break;
        }
        apogee_agl = apogee_agl.max(altitude - pad_altitude_asl);
        feed(estimator, altitude);
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
    let mut estimator = RocketStateEstimator::new(FlightProfile::Dual {
        drogue_chute_minimum_altitude_agl: 500.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: main_agl,
        main_chute_delay_us: 0,
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
    let mut estimator = RocketStateEstimator::new(FlightProfile::Single {
        minimum_deployment_altitude_agl: 500.0,
        drogue_delay_us: 0,
        main_delay_us: 0,
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
    let mut estimator = RocketStateEstimator::new(FlightProfile::Dual {
        drogue_chute_minimum_altitude_agl: 5000.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: 457.2,
        main_chute_delay_us: 0,
    });

    let result = simulate_flight(&mut estimator, 200.0, 40.0, 1.5);
    assert!(result.drogue.is_none());
    assert!(result.main.is_none());
    assert!(matches!(
        result.final_state,
        RocketState::FailedToReachMinApogee
    ));
}
