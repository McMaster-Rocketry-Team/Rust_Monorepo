use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

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
///
/// The one concession: the estimator now requires pad calibration (>= 3
/// surviving 2 s windows) before it will detect ignition, and neither
/// recorder was started minutes before launch the way the real rocket is
/// armed. `extend_pad` loops each log's own genuine pad noise in front to
/// stand in for the longer rail wait; everything from ignition on is the
/// raw log.
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

/// Loop the log's own pad segment (everything up to 0.3 s before ignition,
/// so no spool-up gets copied) in front of the log until at least
/// `extra_s` more pad exists. The copies carry the pad's real noise, sway
/// and timestamp jitter — only the wall-clock length of the rail wait is
/// synthetic. Everything from the original log keeps its raw relative
/// timing (the whole log is just shifted later by the added span).
fn extend_pad(rows: Vec<Measurement>, extra_s: f32) -> Vec<Measurement> {
    let ign_i = find_ignition(&rows);
    let t_ign = rows[ign_i].timestamp_us;
    let pad: Vec<&Measurement> = rows
        .iter()
        .take_while(|r| r.timestamp_us + 300_000 <= t_ign)
        .collect();
    assert!(pad.len() > 2, "no pad segment to loop");
    let span = pad.last().unwrap().timestamp_us - pad[0].timestamp_us;
    let dt_median = {
        let mut dts: Vec<u64> = pad
            .windows(2)
            .map(|w| w[1].timestamp_us - w[0].timestamp_us)
            .collect();
        dts.sort_unstable();
        dts[dts.len() / 2]
    };
    let period = span + dt_median;
    let copies = (extra_s * 1e6 / period as f32).ceil() as u64;
    let mut out = Vec::with_capacity(rows.len() + pad.len() * copies as usize);
    for k in 0..copies {
        for r in &pad {
            out.push(Measurement::new(
                r.timestamp_us + k * period,
                &r.acceleration(),
                &r.angular_velocity(),
                r.altitude_asl(),
            ));
        }
    }
    for r in &rows {
        out.push(Measurement::new(
            r.timestamp_us + copies * period,
            &r.acceleration(),
            &r.angular_velocity(),
            r.altitude_asl(),
        ));
    }
    out
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
        track.push((t, estimator.kf_vertical_velocity().abs()));
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
    apogee_alt_asl: Option<f32>,
    /// (row index, estimated vv) while the filter was alive
    vv_track: Vec<(usize, f32)>,
    /// wall time (s from log start) when vote 1 was first true
    first_v1: Option<f32>,
    /// continuous spans (start s, end s) where vote 3 (baro rate) held true
    v3_spans: Vec<(f32, f32)>,
    /// wall time (s from log start) when pad calibration first completed
    calibration_complete_s: Option<f32>,
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
        apogee_alt_asl: None,
        vv_track: Vec::new(),
        first_v1: None,
        v3_spans: Vec::new(),
        calibration_complete_s: None,
    };
    let mut v3_span_start: Option<f32> = None;
    for (i, z) in rows.iter().enumerate() {
        let speed = deployment.and_then(|t| lookup_speed(t, z.timestamp_us));
        estimator.update(z, speed);

        let now = t_s(rows, i);
        if result.calibration_complete_s.is_none() && estimator.calibration_complete() {
            result.calibration_complete_s = Some(now);
        }
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
            result.apogee_alt_asl = estimator.altitude_asl();
        }
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
/// profile: the pad calibration completes from the screened windows (the
/// real pad has ~12 s and genuine rail sway — the sway windows must be
/// rejected, the quiet ones must carry it), then the filter is born right
/// after thrust alignment, tracks through the ejection blast, and latches
/// apogee with persistence.
#[test]
fn void_lake_v2_replay() {
    init_logger();
    // ~12 s of real pad gives 5 finished windows of which 3 survive the
    // sway screen — exactly the calibration floor. Loop one extra copy of
    // the genuine pad so the test is not balanced on that knife edge.
    let rows = extend_pad(void_lake_rows(), 8.0);
    let (apogee_ref_i, apogee_ref_alt_asl) = baro_apogee(&rows);
    let ign_s = t_s(&rows, find_ignition(&rows));

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

    // Calibration must complete on the pad, before ignition — otherwise
    // ignition detection would have been refused and nothing below runs.
    let cal_s = result
        .calibration_complete_s
        .expect("pad calibration never completed");
    eprintln!("void lake v2: calibration complete at t={cal_s:.1}s (ignition {ign_s:.1}s)");
    assert!(cal_s < ign_s, "calibration completed only at {cal_s}s");

    let (birth_t, forced) = result.birth.expect("filter never born");
    assert!(!forced);
    let birth_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6;
    eprintln!("void lake v2: born at t={birth_s:.1}s");

    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    let apogee_err_m = result.apogee_alt_asl.unwrap() - apogee_ref_alt_asl;
    eprintln!(
        "void lake v2: apogee {:+.1}s / {:+.1}m vs baro ref (ref {:.1} m at t={:.1}s)",
        apogee_err_s,
        apogee_err_m,
        apogee_ref_alt_asl,
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
        // (times from ignition detection) true 0.75 M crossing ~12.6 s,
        // 0.8 M slightly earlier: T_min well before, T_max bounded well
        // before apogee (~32 s after ignition)
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
/// the truth table the plan requires. The recorder started only ~1.8 s
/// before ignition, so the pad is extended with its own looped noise to
/// let calibration complete (see `extend_pad`); all vote/birth times below
/// are measured from ignition, which the extension does not move.
#[test]
fn lc25_v2_replay() {
    init_logger();
    let rows = extend_pad(lc25_rows(), 12.0);
    let (apogee_ref_i, apogee_ref_alt_asl) = baro_apogee(&rows);
    let ign_s = t_s(&rows, find_ignition(&rows));
    let deployment = deployment_speed_track(&rows, lc25_deployment_profile());

    let result = replay(&rows, lc25_config(), Some(&deployment));

    let cal_s = result
        .calibration_complete_s
        .expect("pad calibration never completed");
    assert!(cal_s < ign_s, "calibration completed only at {cal_s}s");

    let (birth_t, forced) = result.birth.expect("filter never born");
    let birth_rel = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;
    let v1_rel = result.first_v1.map(|t| t - ign_s);
    eprintln!(
        "lc25 v2: born at ignition+{birth_rel:.1}s (forced: {forced}), v1 first true {v1_rel:?}, v3 spans (rel) {:?}",
        result
            .v3_spans
            .iter()
            .map(|(s, e)| (s - ign_s, e - ign_s))
            .collect::<Vec<_>>()
    );

    // Vote truth table (times from ignition): the exit must come from the
    // vote, at an honest time — after the genuine supersonic region
    // (baro-truth vv was last above 280 m/s ~10 s after ignition), before
    // T_max (20 s). At VOTE_MACH 0.8 the vote flips roughly a second
    // earlier than the old 0.75 numbers.
    assert!(!forced, "exit degenerated to the T_max timeout");
    assert!(
        (10.5..18.0).contains(&birth_rel),
        "birth at ignition+{birth_rel}s is outside the honest window"
    );
    // Momentary vote-3 flickers in the shock region are expected (the
    // transonic error crosses zero) and harmless — that is why the exit
    // needs 2 votes SUSTAINED. What must never happen is a sustained
    // vote-3 span in the genuinely supersonic region (before ignition
    // +11 s): a second lying vote there would open the lockout.
    for (start, end) in &result.v3_spans {
        if *start - ign_s < 11.0 {
            assert!(
                end - start < 1.0,
                "baro-rate vote held ignition+{}s..{}s while supersonic",
                start - ign_s,
                end - ign_s
            );
        }
    }
    // Vote 1 (inertial speed) must flip near the true 0.8 M crossing
    // (~11.5-12 s after ignition), never while genuinely supersonic
    // (>280 m/s until ~10 s).
    let v1_rel = v1_rel.expect("inertial vote never passed");
    assert!(
        (10.0..15.0).contains(&v1_rel),
        "inertial vote first true at ignition+{v1_rel}s"
    );

    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    let apogee_err_m = result.apogee_alt_asl.unwrap() - apogee_ref_alt_asl;
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
    let mut rows = extend_pad(lc25_rows(), 12.0);
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
    let ign_s = t_s(&rows, find_ignition(&rows));
    let deployment = deployment_speed_track(&rows, lc25_deployment_profile());

    let result = replay(&rows, lc25_config(), Some(&deployment));

    let (birth_t, _forced) = result.birth.expect("filter never born");
    let birth_rel = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;
    eprintln!("lc25 clipped: born at ignition+{birth_rel:.1}s");

    // The under-reading inertial vote alone must not exit while genuinely
    // supersonic (baro-truth vv above 280 m/s until ~10 s after ignition).
    assert!(
        birth_rel > 10.0,
        "clipped accel opened the lockout at ignition+{birth_rel}s — supersonic"
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

/// Powered on seconds before launch: the RAW LC'25 log, whose recorder
/// started only ~1.8 s before ignition — not one 2 s calibration window
/// finishes on the pad. The old fallback would have flown this on a
/// ring-buffer guess with a pessimistic spread; now the pad never
/// calibrates and the estimator must REFUSE to detect ignition: it stays
/// on pad through the entire flight and says so. (Windows collected
/// in-flight must not fake a calibration either — the screening rejects
/// them because the baro/gyro/accel means never agree.) In the real
/// system arming is blocked on `calibration_complete()`, so this flight
/// would never have left the rail.
#[test]
fn short_pad_refuses_ignition() {
    init_logger();
    let rows = lc25_rows();
    let mut estimator = AirbrakesEstimator::new(lc25_config());
    for (i, z) in rows.iter().enumerate() {
        estimator.update(z, None);
        assert!(
            !estimator.calibration_complete(),
            "calibration claimed complete at t={:.1}s on a 1.8 s pad",
            t_s(&rows, i)
        );
    }
    assert!(estimator.birth().is_none(), "filter born without calibration");
    assert!(!estimator.baro_trusted());
    assert!(estimator.altitude_asl().is_none(), "left the pad state");
    assert!(estimator.launch_pad_altitude_asl().is_none());
    assert!(!estimator.is_apogee());
}

// ---------------------------------------------------------------------------
// Diagnostic, not a regression test (run with --ignored --nocapture): how
// accurate is PURE dead-reckoned vertical velocity against the baro-rate
// reference, given a clean start? Three runs:
//   1. LC'25, clean accel, DR from the pad (the flight the estimator flies).
//   2. Void Lake, DR from the pad THROUGH the ±16 g clipping (known bad).
//   3. Void Lake, DR velocity re-seeded from the baro rate right after the
//      last clipped sample (the "start after clipping" question).
// ---------------------------------------------------------------------------

fn q_from_vecs(start: &Vector3<f32>, end: &Vector3<f32>) -> UnitQuaternion<f32> {
    let (s, e) = (start.normalize(), end.normalize());
    let angle = e.angle(&s);
    if angle.to_degrees() < 0.05 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_axis_angle(&UnitVector3::new_normalize(e.cross(&s)), angle)
    }
}

fn dr_diagnostic(
    name: &str,
    rows: &[Measurement],
    // Some(delay_us): re-seed DR velocity from the baro rate this long
    // after the last pre-apogee clipped sample
    restart_after_clip: Option<u64>,
    ref_valid_from_s: f32,
) {
    use super::dead_reckoner::DeadReckoner;
    const G: f32 = 9.81;
    const CLIP: f32 = 0.98 * 16.0 * G;
    let up = Vector3::new(0.0f32, 0.0, 1.0);

    let ign_i = find_ignition(rows);
    let t_ign = rows[ign_i].timestamp_us;

    // Pad calibration from [-2.2 s, -0.2 s] before ignition.
    let (mut acc_sum, mut gyro_sum, mut alt_sum, mut n) =
        (Vector3::<f32>::zeros(), Vector3::<f32>::zeros(), 0.0f32, 0usize);
    for r in rows {
        let back = t_ign.saturating_sub(r.timestamp_us);
        if (200_000..=2_200_000).contains(&back) {
            acc_sum += r.acceleration();
            gyro_sum += r.angular_velocity();
            alt_sum += r.altitude_asl();
            n += 1;
        }
    }
    assert!(n > 100, "{name}: pad window too small ({n})");
    let gravity = acc_sum / n as f32;
    let bias = gyro_sum / n as f32;
    let axis_body = gravity.normalize();

    let mut dr = DeadReckoner::new(q_from_vecs(&up, &gravity));
    dr.position.z = alt_sum / n as f32;

    let (apogee_i, _) = baro_apogee(rows);

    // Last clipped sample BEFORE apogee (boost clipping) — the restart
    // point. Chute-shock/landing clipping after apogee is irrelevant.
    let clips: Vec<usize> = rows[..apogee_i]
        .iter()
        .enumerate()
        .filter(|(_, r)| r.acceleration().abs().max() >= CLIP)
        .map(|(i, _)| i)
        .collect();
    let last_clip = clips.last().copied();
    let clip_count = clips.len();
    let restart_t = match (last_clip, restart_after_clip) {
        (Some(i), Some(delay_us)) => Some(rows[i].timestamp_us + delay_us),
        _ => None,
    };
    eprintln!(
        "=== {name}: ignition t={:.2}s, {clip_count} clipped samples before apogee, apogee ref t={:.2}s",
        t_s(rows, ign_i),
        t_s(rows, apogee_i)
    );
    if let (Some(first), Some(last)) = (clips.first(), clips.last()) {
        eprintln!(
            "    boost clipping spans t={:.2}s .. t={:.2}s",
            t_s(rows, *first),
            t_s(rows, *last)
        );
    }

    let start_i = rows
        .iter()
        .position(|r| r.timestamp_us + 200_000 >= t_ign)
        .unwrap();
    let mut prev_t = rows[start_i].timestamp_us;
    let mut restarted = false;
    let mut next_print = t_ign;
    eprintln!("    {:>6} {:>9} {:>9} {:>8}  (vv m/s)", "t(s)", "DR", "baro-ref", "err");
    for i in start_i + 1..=apogee_i {
        let z = &rows[i];
        let dt = ((z.timestamp_us.saturating_sub(prev_t)) as f32 * 1e-6).clamp(0.0, MAX_DT_S);
        prev_t = z.timestamp_us;
        dr.update(&z.acceleration(), &(z.angular_velocity() - bias), dt);

        if !restarted
            && let Some(t0) = restart_t
            && z.timestamp_us >= t0
        {
            let axis_earth = dr.orientation.transform_vector(&axis_body);
            let tilt = up.angle(&axis_earth);
            let vv_ref = reference_velocity(rows, i);
            dr.velocity = axis_earth * (vv_ref / tilt.cos());
            dr.position.z = z.altitude_asl();
            restarted = true;
            eprintln!(
                "    -- velocity re-seeded at t={:.2}s from baro rate {:.1} m/s (tilt {:.1} deg)",
                t_s(rows, i),
                vv_ref,
                tilt.to_degrees()
            );
        }

        if z.timestamp_us >= next_print || i == apogee_i {
            let vv_ref = reference_velocity(rows, i);
            let err = dr.velocity.z - vv_ref;
            let ref_ok = t_s(rows, i) - t_s(rows, ign_i) >= ref_valid_from_s;
            eprintln!(
                "    {:>6.1} {:>9.1} {:>9.1} {:>+8.1}{}",
                t_s(rows, i),
                dr.velocity.z,
                vv_ref,
                err,
                if ref_ok { "" } else { "  (ref not honest here)" }
            );
            next_print += 2_000_000;
        }
    }
    eprintln!(
        "    at apogee: DR vv {:+.1} m/s (truth 0), DR altitude {:.0} m vs baro ref {:.0} m",
        dr.velocity.z,
        dr.position.z,
        rows[apogee_i].altitude_asl()
    );
}

#[test]
#[ignore]
fn dr_velocity_accuracy_diagnostic() {
    init_logger();
    let lc25 = lc25_rows();
    dr_diagnostic("LC'25 clean, DR from pad", &lc25, None, 13.0);
    let vl = void_lake_rows();
    dr_diagnostic("Void Lake, DR from pad through clipping", &vl, None, 0.0);
    dr_diagnostic(
        "Void Lake, DR re-seeded 0.2s after last clip",
        &vl,
        Some(200_000),
        0.0,
    );
    dr_diagnostic(
        "Void Lake, DR re-seeded 3s after last clip (baro honest)",
        &vl,
        Some(3_000_000),
        0.0,
    );
}

/// Ignition index: first sample whose 0.1 s rolling mean of |acc| exceeds
/// 4 g.
fn find_ignition(rows: &[Measurement]) -> usize {
    let (mut lo, mut sum, mut cnt) = (0usize, 0.0f32, 0usize);
    for i in 0..rows.len() {
        sum += rows[i].acceleration().magnitude();
        cnt += 1;
        while rows[i].timestamp_us - rows[lo].timestamp_us > 100_000 {
            sum -= rows[lo].acceleration().magnitude();
            cnt -= 1;
            lo += 1;
        }
        if cnt > 10 && sum / cnt as f32 > 4.0 * 9.81 {
            return i;
        }
    }
    panic!("no ignition found");
}

fn series_std(v: &[f32]) -> f32 {
    let n = v.len() as f32;
    let mean = v.iter().sum::<f32>() / n;
    (v.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>() / (n - 1.0)).sqrt()
}

/// Sample-to-sample (high-frequency) noise: std of first differences over
/// sqrt(2). Insensitive to slow real motion (rail sway, weather drift) —
/// this is the sensor's own white-noise floor.
fn hf_noise(v: &[f32]) -> f32 {
    let d: Vec<f32> = v.windows(2).map(|w| w[1] - w[0]).collect();
    series_std(&d) / 2f32.sqrt()
}

/// Std of residuals around a least-squares line — removes linear drift
/// (baro weather trend) but keeps everything faster.
fn detrended_std(t_s: &[f32], v: &[f32]) -> f32 {
    let n = v.len() as f32;
    let tm = t_s.iter().sum::<f32>() / n;
    let vm = v.iter().sum::<f32>() / n;
    let (mut num, mut den) = (0.0f32, 0.0f32);
    for (t, x) in t_s.iter().zip(v) {
        num += (t - tm) * (x - vm);
        den += (t - tm) * (t - tm);
    }
    let slope = if den > 0.0 { num / den } else { 0.0 };
    let res: Vec<f32> = t_s
        .iter()
        .zip(v)
        .map(|(t, x)| x - (vm + slope * (t - tm)))
        .collect();
    series_std(&res)
}

fn pad_noise(name: &str, rows: &[Measurement], window_s: f32) {
    let ign_i = find_ignition(rows);
    let t_ign = rows[ign_i].timestamp_us;
    // Stay 0.3 s clear of the detection point (motor spool-up + detection
    // lag pollute the tail).
    let end_back = 300_000u64;
    let start_back = end_back + (window_s * 1e6) as u64;
    let win: Vec<&Measurement> = rows
        .iter()
        .filter(|r| {
            let back = t_ign.saturating_sub(r.timestamp_us);
            back >= end_back && back <= start_back && r.timestamp_us < t_ign
        })
        .collect();
    assert!(win.len() > 200, "{name}: only {} pad samples", win.len());

    let mut dts: Vec<f32> = win
        .windows(2)
        .map(|w| (w[1].timestamp_us - w[0].timestamp_us) as f32 * 1e-6)
        .collect();
    dts.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    eprintln!(
        "=== {name}: {} samples over {:.1}s (median dt {:.2} ms, max {:.1} ms)",
        win.len(),
        window_s,
        dts[dts.len() / 2] * 1e3,
        dts.last().unwrap() * 1e3
    );

    let t0 = win[0].timestamp_us;
    let ts: Vec<f32> = win
        .iter()
        .map(|r| (r.timestamp_us - t0) as f32 * 1e-6)
        .collect();
    for (axis, label) in [(0usize, "x"), (1, "y"), (2, "z")] {
        let acc: Vec<f32> = win.iter().map(|r| r.acceleration()[axis]).collect();
        let gyro: Vec<f32> = win
            .iter()
            .map(|r| r.angular_velocity()[axis].to_degrees())
            .collect();
        eprintln!(
            "    acc {label}: std {:7.4} m/s2 (hf {:7.4})    gyro {label}: std {:7.4} deg/s (hf {:7.4})",
            series_std(&acc),
            hf_noise(&acc),
            series_std(&gyro),
            hf_noise(&gyro),
        );
    }
    let alt: Vec<f32> = win.iter().map(|r| r.altitude_asl()).collect();
    eprintln!(
        "    baro altitude: detrended std {:.3} m (hf {:.3} m)",
        detrended_std(&ts, &alt),
        hf_noise(&alt)
    );
}

#[test]
#[ignore]
fn pad_sensor_noise_comparison() {
    init_logger();
    let lc25 = lc25_rows();
    let vl = void_lake_rows();
    // LC'25 has only ~1.8 s before ignition -> 1.4 s usable window; measure
    // Void Lake over the same window length for a fair comparison, plus its
    // full pad span for context.
    pad_noise("LC'25 (COTS recorder, 500 Hz), 1.4 s window", &lc25, 1.4);
    pad_noise("Void Lake (VLF5 LSM6DSM, 416 Hz), 1.4 s window", &vl, 1.4);
    pad_noise("Void Lake (VLF5 LSM6DSM, 416 Hz), 10 s window", &vl, 10.0);
}
