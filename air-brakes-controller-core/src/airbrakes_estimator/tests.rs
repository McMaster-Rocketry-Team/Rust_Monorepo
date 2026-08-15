use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::Vector3;

use super::*;
use crate::{
    baro_state_estimator::{DeploymentProfile, FlightProfile, RocketStateEstimator},
    tests::init_logger,
};

/// Both replays feed the estimator the RAW timestamped log — no resampling
/// onto a perfect grid. This is deliberate (red-team finding): the Void
/// Lake log has 104 ms sensor stalls and the LC'25 recorder runs at
/// 500 Hz, and the estimator must handle both through its measured-dt
/// path. A test suite that resamples first can never catch a hidden
/// "assumes 416 Hz" bug.
fn void_lake_rows() -> Vec<Measurement> {
    #[derive(serde::Deserialize)]
    struct CsvRow {
        timestamp_us: u64,
        acc_x: f32,
        acc_y: f32,
        acc_z: f32,
        gyro_x: f32,
        gyro_y: f32,
        gyro_z: f32,
        pressure: f32,
        imu_valid: bool,
        baro_valid: bool,
    }

    let mut reader = csv::Reader::from_path("./test_data/void_lake_flight.csv").unwrap();
    reader
        .deserialize::<CsvRow>()
        .map(|r| r.unwrap())
        .filter(|r| r.imu_valid && r.baro_valid && r.pressure > 10_000.0)
        .map(|r| {
            Measurement::new(
                r.timestamp_us,
                &Vector3::new(r.acc_x, r.acc_y, r.acc_z),
                // VLF5 logs gyro in deg/s
                &Vector3::new(
                    r.gyro_x.to_radians(),
                    r.gyro_y.to_radians(),
                    r.gyro_z.to_radians(),
                ),
                calculate_isa_altitude(Pascals(r.pressure as f64)).0 as f32,
            )
        })
        .collect()
}

fn lc25_rows() -> Vec<Measurement> {
    #[derive(serde::Deserialize)]
    struct CsvRow {
        time_us: u64,
        imu_acc_x: f32,
        imu_acc_y: f32,
        imu_acc_z: f32,
        gyro_x: f32,
        gyro_y: f32,
        gyro_z: f32,
        altitude: f32,
    }
    let mut reader = csv::Reader::from_path("./test_data/lc_25.csv").unwrap();
    reader
        .deserialize::<CsvRow>()
        .map(|r| r.unwrap())
        .map(|r| {
            Measurement::new(
                r.time_us,
                // this recorder's acc y/z are sign-flipped relative to its
                // gyro frame
                &Vector3::new(r.imu_acc_x, -r.imu_acc_y, -r.imu_acc_z),
                &Vector3::new(
                    r.gyro_x.to_radians(),
                    r.gyro_y.to_radians(),
                    r.gyro_z.to_radians(),
                ),
                r.altitude,
            )
        })
        .collect()
}

/// Baro-derived vertical velocity reference: central difference over
/// +-0.5 s of real timestamps. Honest where the baro is honest (subsonic,
/// low dynamic pressure).
fn reference_velocity(rows: &[Measurement], i: usize) -> f32 {
    let t = rows[i].timestamp_us;
    let half_us = 500_000u64;
    let mut lo = i;
    while lo > 0 && t.saturating_sub(rows[lo - 1].timestamp_us) <= half_us {
        lo -= 1;
    }
    let mut hi = i;
    while hi + 1 < rows.len() && rows[hi + 1].timestamp_us.saturating_sub(t) <= half_us {
        hi += 1;
    }
    let dt = (rows[hi].timestamp_us - rows[lo].timestamp_us) as f32 * 1e-6;
    if dt <= 0.0 {
        return 0.0;
    }
    (rows[hi].altitude_asl() - rows[lo].altitude_asl()) / dt
}

/// (index, altitude) of the highest 1 s-smoothed baro altitude.
fn baro_apogee(rows: &[Measurement]) -> (usize, f32) {
    let mut best = (0usize, f32::MIN);
    for i in 0..rows.len() {
        // cheap smoothing: skip the raw-sample extremes by averaging a
        // small neighborhood
        if i < 50 || i + 50 >= rows.len() {
            continue;
        }
        let mut sum = 0.0f32;
        let mut n = 0;
        for j in (i - 50)..(i + 50) {
            sum += rows[j].altitude_asl();
            n += 1;
        }
        let avg = sum / n as f32;
        if avg > best.1 {
            best = (i, avg);
        }
    }
    best
}

fn t_s(rows: &[Measurement], i: usize) -> f32 {
    (rows[i].timestamp_us - rows[0].timestamp_us) as f32 * 1e-6
}

/// Run the slow deployment estimator over the log (it is sample-clocked at
/// 416 Hz, so feed it a linearly resampled stream) and record its speed at
/// each wall time — this is vote 2 of the lockout exit, exactly as the
/// firmware will provide it.
fn deployment_speed_track(rows: &[Measurement], profile: FlightProfile) -> Vec<(u64, f32)> {
    let mut estimator = RocketStateEstimator::new(profile);
    let dt_us = (1_000_000f64 / 416.0) as u64;
    let mut track = Vec::new();
    let t0 = rows[0].timestamp_us;
    let t_end = rows.last().unwrap().timestamp_us;
    let mut t = t0;
    let mut j = 0usize;
    while t <= t_end {
        while j + 1 < rows.len() && rows[j + 1].timestamp_us <= t {
            j += 1;
        }
        let alt = if j + 1 < rows.len() {
            let (ta, tb) = (rows[j].timestamp_us, rows[j + 1].timestamp_us);
            let frac = (t - ta) as f32 / (tb - ta).max(1) as f32;
            rows[j].altitude_asl() + (rows[j + 1].altitude_asl() - rows[j].altitude_asl()) * frac
        } else {
            rows[j].altitude_asl()
        };
        let _ = estimator.update(alt);
        track.push((t, estimator.vertical_velocity().abs()));
        t += dt_us;
    }
    track
}

fn lookup_speed(track: &[(u64, f32)], t: u64) -> Option<f32> {
    match track.binary_search_by_key(&t, |e| e.0) {
        Ok(i) => Some(track[i].1),
        Err(0) => None,
        Err(i) => Some(track[i - 1].1),
    }
}

struct ReplayResult {
    birth: Option<(u64, bool)>,
    apogee_i: Option<usize>,
    apogee_alt: Option<f32>,
    /// (row index, estimated vv) while the filter was alive
    vv_track: Vec<(usize, f32)>,
    /// wall time (s from log start) when vote 1 was first true
    first_v1: Option<f32>,
    /// continuous spans (start s, end s) where vote 3 (baro rate) held true
    v3_spans: Vec<(f32, f32)>,
    clipped: u32,
}

fn replay(
    rows: &[Measurement],
    config: AirbrakesConfig,
    deployment: Option<&[(u64, f32)]>,
) -> ReplayResult {
    let mut estimator = AirbrakesEstimator::new(config);
    let mut result = ReplayResult {
        birth: None,
        apogee_i: None,
        apogee_alt: None,
        vv_track: Vec::new(),
        first_v1: None,
        v3_spans: Vec::new(),
        clipped: 0,
    };
    let mut v3_span_start: Option<f32> = None;
    for (i, z) in rows.iter().enumerate() {
        let speed = deployment.and_then(|t| lookup_speed(t, z.timestamp_us));
        estimator.update(z, speed);

        let now = t_s(rows, i);
        if let Some((v1, _, v3)) = estimator.lockout_votes() {
            if v1 && result.first_v1.is_none() {
                result.first_v1 = Some(now);
            }
            match (v3, v3_span_start) {
                (true, None) => v3_span_start = Some(now),
                (false, Some(start)) => {
                    result.v3_spans.push((start, now));
                    v3_span_start = None;
                }
                _ => {}
            }
        } else if let Some(start) = v3_span_start.take() {
            result.v3_spans.push((start, now));
        }
        if result.birth.is_none() {
            result.birth = estimator.birth();
        }
        if let Some(v) = estimator.velocity() {
            result.vv_track.push((i, v.y));
        }
        if estimator.is_apogee() && result.apogee_i.is_none() {
            result.apogee_i = Some(i);
            result.apogee_alt = estimator.altitude_asl();
        }
        result.clipped = result.clipped.max(estimator.accel_clipped_samples());
    }
    result
}

/// Mean |vv error| vs the baro-rate reference over rows [from, to).
fn vv_error(rows: &[Measurement], track: &[(usize, f32)], from: usize, to: usize) -> (f32, usize) {
    let mut sum = 0.0f32;
    let mut n = 0usize;
    for (i, vv) in track {
        if *i >= from && *i < to {
            sum += (vv - reference_velocity(rows, *i)).abs();
            n += 1;
        }
    }
    (if n > 0 { sum / n as f32 } else { f32::NAN }, n)
}

/// Void Lake, RAW timestamps (real 104 ms stalls included), subsonic
/// profile: the filter is born right after thrust alignment, tracks
/// through the ejection blast, and latches apogee with persistence. The
/// accel clipped at the ±16 g rail on this flight — the counter must see
/// it.
#[test]
fn void_lake_v2_replay() {
    init_logger();
    let rows = void_lake_rows();
    let (apogee_ref_i, apogee_ref_alt) = baro_apogee(&rows);

    let result = replay(
        &rows,
        AirbrakesConfig {
            ignition_detection_acc_threshold: 4.0 * 9.81,
            mach_lockout: None,
            // Void Lake airframe port coefficient from the drag-model
            // closure analysis (order 2-3e-3)
            baro_port_coefficient: 2.5e-3,
        },
        None,
    );

    let (birth_t, forced) = result.birth.expect("filter never born");
    assert!(!forced);
    let birth_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6;
    eprintln!("void lake v2: born at t={birth_s:.1}s");

    // The accel hit the ±16 g rail during boost on this flight
    assert!(result.clipped > 0, "clip counter saw nothing");
    eprintln!("void lake v2: {} clipped accel samples", result.clipped);

    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    let apogee_err_m = result.apogee_alt.unwrap() - apogee_ref_alt;
    eprintln!(
        "void lake v2: apogee {:+.1}s / {:+.1}m vs baro ref (ref {:.1} m at t={:.1}s)",
        apogee_err_s,
        apogee_err_m,
        apogee_ref_alt,
        t_s(&rows, apogee_ref_i)
    );
    assert!(apogee_err_s.abs() < 3.0, "apogee time err {apogee_err_s}");
    assert!(apogee_err_m.abs() < 80.0, "apogee alt err {apogee_err_m}");

    // Coast vertical velocity vs the honest late-coast baro rate: last
    // 10 s before apogee, stopping 2 s short of the blast.
    let mut from = apogee_ref_i;
    while from > 0 && t_s(&rows, apogee_ref_i) - t_s(&rows, from) < 10.0 {
        from -= 1;
    }
    let mut to = apogee_ref_i;
    while to > 0 && t_s(&rows, apogee_ref_i) - t_s(&rows, to) < 2.0 {
        to -= 1;
    }
    let (err, n) = vv_error(&rows, &result.vv_track, from, to);
    eprintln!("void lake v2: coast vv mean |err|={err:.2} m/s over {n} samples");
    assert!(n > 1000, "filter not alive through coast");
    assert!(err < 10.0, "coast vv err {err}");
}

fn lc25_config() -> AirbrakesConfig {
    AirbrakesConfig {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        // ignition ~2 s, true 0.75 M crossing ~14.6 s: T_min well before,
        // T_max bounded well before apogee (33.8 s)
        mach_lockout: Some(MachLockoutConfig {
            t_min_us: 8_000_000,
            t_max_us: 20_000_000,
        }),
        // measured on this flight
        baro_port_coefficient: 0.7e-3,
    }
}

fn lc25_deployment_profile() -> FlightProfile {
    FlightProfile {
        // the slow filter's own timed lockout: from its baro ignition
        // detection (~3 s) until past the true subsonic crossing
        mach_lockout_duration_us: Some(12_000_000),
        max_burn_time_us: 5_000_000,
        deployment: DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: 500.0,
            drogue_chute_delay_us: 0,
            main_chute_altitude_agl: 300.0,
            main_chute_delay_us: 0,
        },
    }
}

/// LC'25, RAW 500 Hz timestamps, Mach 2 profile: the vote (not the timer)
/// must birth the filter, at an honest time — and the vote flip times form
/// the truth table the plan requires.
#[test]
fn lc25_v2_replay() {
    init_logger();
    let rows = lc25_rows();
    let (apogee_ref_i, apogee_ref_alt) = baro_apogee(&rows);
    let deployment = deployment_speed_track(&rows, lc25_deployment_profile());

    let result = replay(&rows, lc25_config(), Some(&deployment));

    let (birth_t, forced) = result.birth.expect("filter never born");
    let birth_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6;
    eprintln!(
        "lc25 v2: born at t={birth_s:.1}s (forced: {forced}), v1 first true {:?}, v3 spans {:?}",
        result.first_v1, result.v3_spans
    );

    // Vote truth table: the exit must come from the vote, at an honest
    // time — after the genuine supersonic region (baro-truth vv was last
    // above 280 m/s at ~12 s), before T_max (ignition ~2 s + 20 s).
    assert!(!forced, "exit degenerated to the T_max timeout");
    assert!(
        (13.5..20.0).contains(&birth_s),
        "birth at {birth_s}s is outside the honest window"
    );
    // Momentary vote-3 flickers in the shock region are expected (the
    // transonic error crosses zero) and harmless — that is why the exit
    // needs 2 votes SUSTAINED. What must never happen is a sustained
    // vote-3 span in the genuinely supersonic region (before ~13 s): a
    // second lying vote there would open the lockout.
    for (start, end) in &result.v3_spans {
        if *start < 13.0 {
            assert!(
                end - start < 1.0,
                "baro-rate vote held {}s..{}s while supersonic",
                start,
                end
            );
        }
    }
    // Vote 1 (inertial speed) must flip near the true 0.75 M crossing
    // (~14.6 s), never while supersonic.
    let v1_t = result.first_v1.expect("inertial vote never passed");
    assert!(
        (13.0..17.0).contains(&v1_t),
        "inertial vote first true at {v1_t}s"
    );

    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    let apogee_err_m = result.apogee_alt.unwrap() - apogee_ref_alt;
    eprintln!("lc25 v2: apogee {apogee_err_s:+.1}s / {apogee_err_m:+.1}m vs baro ref");
    assert!(apogee_err_s.abs() < 3.0, "apogee time err {apogee_err_s}");
    assert!(apogee_err_m.abs() < 80.0, "apogee alt err {apogee_err_m}");

    // Coast accuracy from 2 s after birth to 2 s before apogee.
    let from = rows
        .iter()
        .position(|r| r.timestamp_us > birth_t + 2_000_000)
        .unwrap();
    let mut to = apogee_ref_i;
    while to > 0 && t_s(&rows, apogee_ref_i) - t_s(&rows, to) < 2.0 {
        to -= 1;
    }
    let (err, n) = vv_error(&rows, &result.vv_track, from, to);
    eprintln!("lc25 v2: coast vv mean |err|={err:.2} m/s over {n} samples");
    assert!(n > 1000);
    assert!(err < 10.0, "coast vv err {err}");
}

/// LC'25 with the accelerometer artificially clipped at ±16 g (the Void
/// Lake failure, injected into the Mach 2 flight): the inertial vote lies
/// low and early, but a single lying vote must NOT open the lockout — and
/// after birth the baro must pull the wrong dead-reckoned velocity back.
#[test]
fn lc25_clipped_accel_replay() {
    init_logger();
    let mut rows = lc25_rows();
    const RAIL: f32 = 16.0 * 9.81;
    for r in &mut rows {
        let acc = r.acceleration().map(|a| a.clamp(-RAIL, RAIL));
        *r = Measurement::new(
            r.timestamp_us,
            &acc,
            &r.angular_velocity(),
            r.altitude_asl(),
        );
    }
    let (apogee_ref_i, _) = baro_apogee(&rows);
    let deployment = deployment_speed_track(&rows, lc25_deployment_profile());

    let result = replay(&rows, lc25_config(), Some(&deployment));

    assert!(result.clipped > 0, "clip injection did not register");
    let (birth_t, _forced) = result.birth.expect("filter never born");
    let birth_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6;
    eprintln!(
        "lc25 clipped: born at t={birth_s:.1}s, {} clipped samples",
        result.clipped
    );

    // The under-reading inertial vote alone must not exit while genuinely
    // supersonic (true crossing ~14.6 s).
    assert!(
        birth_s > 13.0,
        "clipped accel opened the lockout at {birth_s}s — supersonic"
    );

    // Within 5 s of birth the baro must have pulled vv to within 15 m/s
    // of the honest reference (the born/reanchor velocity variance is
    // what makes this fast).
    let check_t = birth_t + 5_000_000;
    let check_i = rows.iter().position(|r| r.timestamp_us > check_t).unwrap();
    let vv_est = result
        .vv_track
        .iter()
        .filter(|(i, _)| *i >= check_i)
        .map(|(_, v)| *v)
        .next()
        .expect("filter died after birth");
    let vv_ref = reference_velocity(&rows, check_i);
    eprintln!("lc25 clipped: vv {vv_est:.1} vs ref {vv_ref:.1} at birth+5s");
    assert!(
        (vv_est - vv_ref).abs() < 15.0,
        "vv did not recover: {vv_est} vs ref {vv_ref}"
    );

    // Apogee must still be called sanely.
    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    eprintln!("lc25 clipped: apogee {apogee_err_s:+.1}s vs baro ref");
    assert!(apogee_err_s.abs() < 4.0, "apogee time err {apogee_err_s}");
}
