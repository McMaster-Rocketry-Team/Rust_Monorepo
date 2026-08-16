use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

use super::*;
use crate::{
    controller::RocketParameters,
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

struct ReplayResult {
    birth: Option<(u64, bool)>,
    apogee_i: Option<usize>,
    apogee_alt_asl: Option<f32>,
    /// (row index, estimated vv) while the filter was alive
    vv_track: Vec<(usize, f32)>,
    /// continuous spans (start s, end s) where the drag vote held true
    vote_spans: Vec<(f32, f32)>,
    /// wall time (s from log start) when `burnout_detected()` first went true
    burnout_s: Option<f32>,
    /// set if `burnout_detected()` ever went back to false after latching
    burnout_unlatched: bool,
    /// wall time (s from log start) when pad calibration first completed
    calibration_complete_s: Option<f32>,
}

fn replay(rows: &[Measurement], config: AirbrakesConfig) -> ReplayResult {
    let mut estimator = AirbrakesEstimator::new(config);
    let mut result = ReplayResult {
        birth: None,
        apogee_i: None,
        apogee_alt_asl: None,
        vv_track: Vec::new(),
        vote_spans: Vec::new(),
        burnout_s: None,
        burnout_unlatched: false,
        calibration_complete_s: None,
    };
    let mut vote_span_start: Option<f32> = None;
    for (i, z) in rows.iter().enumerate() {
        estimator.update(z);

        let now = t_s(rows, i);
        match (estimator.burnout_detected(), result.burnout_s) {
            (true, None) => result.burnout_s = Some(now),
            (false, Some(_)) => result.burnout_unlatched = true,
            _ => {}
        }
        if result.calibration_complete_s.is_none() && estimator.calibration_complete() {
            result.calibration_complete_s = Some(now);
        }
        match (estimator.lockout_vote(), vote_span_start) {
            (Some(true), None) => vote_span_start = Some(now),
            (Some(true), Some(_)) => {}
            (_, Some(start)) => {
                result.vote_spans.push((start, now));
                vote_span_start = None;
            }
            _ => {}
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
            // Subsonic profile: the drag vote is never consulted, so the
            // airframe cannot affect this run.
            rocket: lc25_rocket(),
        },
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

/// The airframe the drag vote inverts. `cd[0] * reference_area /
/// burnout_mass` = 2.4e-4, which the flight itself corroborates: measured
/// drag deceleration over dynamic pressure sits at 0.00022-0.00026 across
/// the whole subsonic coast (see `mach_detection_signals`).
/// The property the burnout latch exists for, on BOTH profiles: the
/// vertical filter — and therefore the MPC state, and therefore any chance
/// of the brakes opening — must never come alive while the motor is
/// burning.
///
/// This is the airbrakes half's own guarantee. It used to be borrowed from
/// the deployment estimator's `is_coasting()` burn timer, which the open
/// gate consulted; the subsonic path had no thrust check of its own at all
/// and was born ~0.8 s BEFORE burnout (Void Lake: born ignition+0.9 s,
/// burnout +1.7 s). Now both paths are gated on the measured axial-sign
/// latch and the gate needs no cross-half input.
#[test]
fn filter_is_never_born_under_thrust() {
    init_logger();
    for (name, rows, config) in [
        (
            "Void Lake (subsonic profile)",
            extend_pad(void_lake_rows(), 8.0),
            AirbrakesConfig {
                ignition_detection_acc_threshold: 4.0 * 9.81,
                mach_lockout: None,
                rocket: lc25_rocket(),
            },
        ),
        ("LC'25 (supersonic profile)", extend_pad(lc25_rows(), 12.0), lc25_config()),
    ] {
        let ign_s = t_s(&rows, find_ignition(&rows));
        let burnout_s = t_s(&rows, find_burnout(&rows)) - ign_s;

        let result = replay(&rows, config);
        let (birth_t, forced) = result.birth.expect("filter never born");
        let birth_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;

        eprintln!(
            "{name}: burnout at ignition+{burnout_s:.2}s, filter born at \
             ignition+{birth_s:.2}s (forced: {forced})"
        );
        assert!(
            birth_s > burnout_s,
            "{name}: filter born at ignition+{birth_s:.2}s, BEFORE burnout at \
             ignition+{burnout_s:.2}s — the MPC could be handed a state under thrust"
        );

        // The latch is what the log records, so it has to be honest: one
        // way, and reported consistently once the estimator leaves the
        // dead-reckoning state that owns the flag.
        let latched_s = result.burnout_s.expect("burnout never detected") - ign_s;
        assert!(
            !result.burnout_unlatched,
            "{name}: burnout_detected() went back to false after latching"
        );
        // `<=` not `<`: on the subsonic path the baro ring is already full
        // by the time the latch fires, so the same `update` call that
        // latches burnout also births the filter.
        assert!(
            latched_s > burnout_s && latched_s <= birth_s,
            "{name}: latch at ignition+{latched_s:.2}s is not between burnout \
             ({burnout_s:.2}s) and birth ({birth_s:.2}s)"
        );
        eprintln!("{name}: burnout latch reported at ignition+{latched_s:.2}s");
    }
}

fn lc25_rocket() -> RocketParameters {
    RocketParameters {
        burnout_mass: 17.607,
        cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
        reference_area: 0.008982476,
    }
}

fn lc25_config() -> AirbrakesConfig {
    AirbrakesConfig {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        // (times from ignition detection) true 0.75 M crossing ~12.6 s,
        // 0.8 M slightly earlier: T_min well before, T_max bounded well
        // before apogee (~32 s after ignition)
        mach_lockout: Some(MachLockoutConfig {
            earliest_subsonic_after_ignition_us: 8_000_000,
            force_birth_after_ignition_us: 20_000_000,
        }),
        rocket: lc25_rocket(),
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
    let result = replay(&rows, lc25_config());

    let cal_s = result
        .calibration_complete_s
        .expect("pad calibration never completed");
    assert!(cal_s < ign_s, "calibration completed only at {cal_s}s");

    let (birth_t, forced) = result.birth.expect("filter never born");
    let birth_rel = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;
    eprintln!(
        "lc25 v2: born at ignition+{birth_rel:.1}s (forced: {forced}), drag-vote spans (rel) {:?}",
        result
            .vote_spans
            .iter()
            .map(|(s, e)| (s - ign_s, e - ign_s))
            .collect::<Vec<_>>()
    );

    // The exit must come from the drag measurement, not the T_max timer,
    // and at an honest time: after the genuine supersonic region (baro-truth
    // vv was last above 280 m/s ~10 s after ignition) and before T_max.
    assert!(!forced, "exit degenerated to the T_max timeout");
    assert!(
        (10.5..18.0).contains(&birth_rel),
        "birth at ignition+{birth_rel}s is outside the honest window"
    );

    // The property that makes a single vote sufficient: the drag-inverted
    // speed must NEVER read subsonic while the airframe is genuinely
    // supersonic (before ignition+11 s). Not "not for long" — never. The
    // old baro-rate vote could not promise this: with the static-port
    // correction removed it held a full second at ignition+10.4s.
    // Momentary flickers are not tolerated either, because there is no
    // second vote left to out-vote one.
    for (start, end) in &result.vote_spans {
        assert!(
            start - ign_s >= 11.0,
            "drag vote read subsonic at ignition+{}s..{}s, while still supersonic",
            start - ign_s,
            end - ign_s
        );
    }

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
    let result = replay(&rows, lc25_config());

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
        estimator.update(z);
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

/// True apogee: the first sample after burnout where the +-0.5 s baro rate
/// crosses from climbing to falling. Blast-proof, unlike `baro_apogee` —
/// on LC'25 that helper's 1 s-smoothed maximum lands on the post-ejection
/// pressure spike ~2.7 s late, where the baro rate is already -31 m/s.
fn true_apogee(rows: &[Measurement]) -> usize {
    let ign_i = find_ignition(rows);
    let mut climbed = false;
    for i in ign_i..rows.len() {
        let rate = reference_velocity(rows, i);
        if rate > 50.0 {
            climbed = true;
        }
        if climbed && rate <= 0.0 {
            return i;
        }
    }
    panic!("no apogee found");
}

/// The accelerometer channel over the coast, as regressors for the port
/// fit below: the earth-frame vertical acceleration integrated once (V)
/// and twice (Z) from the start of the fit window, plus the tilt needed to
/// turn a vertical velocity into an airspeed.
struct CoastBasis {
    /// s since the start of the fit window
    tau: Vec<f32>,
    /// integral of the earth-frame vertical acceleration (m/s)
    v_int: Vec<f32>,
    /// double integral (m)
    z_int: Vec<f32>,
    cos_tilt: Vec<f32>,
    from_i: usize,
    apogee_i: usize,
    /// If false, airspeed is taken as the plain vertical velocity. The
    /// gyro-only tilt drifts badly late in a long coast (on LC'25 it walks
    /// into the 80 deg cap, where 1/cos inflates the airspeed 6x while the
    /// true speed is nearly zero), so the tilt-corrected regressor is the
    /// suspect one, not the reference.
    use_tilt: bool,
}

impl CoastBasis {
    fn tilt_gain(&self, k: usize) -> f32 {
        if self.use_tilt {
            1.0 / self.cos_tilt[k]
        } else {
            1.0
        }
    }
}

fn coast_basis(rows: &[Measurement], from_i: usize, apogee_i: usize) -> CoastBasis {
    use super::dead_reckoner::DeadReckoner;
    let up = Vector3::new(0.0f32, 0.0, 1.0);

    let ign_i = find_ignition(rows);
    let t_ign = rows[ign_i].timestamp_us;

    // Pad calibration from [-2.2 s, -0.2 s] before ignition (the same
    // window the DR diagnostic uses).
    let (mut acc_sum, mut gyro_sum, mut n) =
        (Vector3::<f32>::zeros(), Vector3::<f32>::zeros(), 0usize);
    for r in rows {
        let back = t_ign.saturating_sub(r.timestamp_us);
        if (200_000..=2_200_000).contains(&back) {
            acc_sum += r.acceleration();
            gyro_sum += r.angular_velocity();
            n += 1;
        }
    }
    assert!(n > 100, "pad window too small ({n})");
    let gravity = acc_sum / n as f32;
    let bias = gyro_sum / n as f32;
    let axis_body = gravity.normalize();

    // Forward pass from the pad: gyro-only attitude gives the earth-frame
    // vertical acceleration and the airframe tilt at every row.
    let mut dr = DeadReckoner::new(q_from_vecs(&up, &gravity));
    let (mut tau, mut v_int, mut z_int, mut cos_tilt) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut prev_t = rows[0].timestamp_us;
    let (mut v, mut z) = (0.0f32, 0.0f32);
    for (i, sample) in rows.iter().enumerate() {
        let dt = ((sample.timestamp_us.saturating_sub(prev_t)) as f32 * 1e-6).clamp(0.0, MAX_DT_S);
        prev_t = sample.timestamp_us;
        dr.update(&sample.acceleration(), &(sample.angular_velocity() - bias), dt);
        if i < from_i {
            continue;
        }
        if i > from_i {
            z += v * dt + 0.5 * dr.acceleration.z * dt * dt;
            v += dr.acceleration.z * dt;
        }
        tau.push((sample.timestamp_us - rows[from_i].timestamp_us) as f32 * 1e-6);
        v_int.push(v);
        z_int.push(z);
        let axis_earth = dr.orientation.transform_vector(&axis_body);
        // the same 80 deg cap the estimator applies before 1/cos blows up
        cos_tilt.push(up.angle(&axis_earth).min(1.396).cos());
    }

    CoastBasis {
        tau,
        v_int,
        z_int,
        cos_tilt,
        from_i,
        apogee_i,
        use_tilt: false,
    }
}

/// Fit only the coast entry state `(z0, v0)` — and optionally an
/// accelerometer bias — with the port coefficient held FIXED at `c`, then
/// report the residual RMS against the raw baro. This is the sweep that
/// needs no collinearity argument: whichever `c` the flight really has is
/// the one that minimises this curve.
fn fixed_c_residual(
    rows: &[Measurement],
    basis: &CoastBasis,
    c: f32,
    with_bias: bool,
) -> (f32, f32) {
    let n_par = 2 + with_bias as usize;
    let mut ata = vec![0.0f64; n_par * n_par];
    let mut atb = vec![0.0f64; n_par];
    let mut n = 0usize;
    let row = |k: usize| -> [f64; 3] {
        let tau = basis.tau[k] as f64;
        [1.0, tau, 0.5 * tau * tau]
    };
    // The fixed port term moves to the left-hand side.
    let target = |k: usize| -> f64 {
        let i = basis.from_i + k;
        let vv = basis.v_int[k] * basis.tilt_gain(k);
        (rows[i].altitude_asl() - basis.z_int[k] - c * vv * vv) as f64
    };
    for k in 0..basis.tau.len() {
        if basis.from_i + k > basis.apogee_i {
            break;
        }
        let r = row(k);
        let y = target(k);
        for a in 0..n_par {
            for b in 0..n_par {
                ata[a * n_par + b] += r[a] * r[b];
            }
            atb[a] += r[a] * y;
        }
        n += 1;
    }
    let x = solve_normal(&mut ata, &mut atb, n_par);
    let mut sse = 0.0f64;
    for k in 0..basis.tau.len() {
        if basis.from_i + k > basis.apogee_i {
            break;
        }
        let r = row(k);
        let pred: f64 = (0..n_par).map(|a| r[a] * x[a]).sum();
        let d = target(k) - pred;
        sse += d * d;
    }
    let k_a = basis.apogee_i - basis.from_i;
    let b = if with_bias { x[2] as f32 } else { 0.0 };
    let v_apogee = x[1] as f32 + basis.v_int[k_a] + b * basis.tau[k_a];
    ((sse / n as f64).sqrt() as f32, v_apogee)
}

/// Gaussian elimination with partial pivoting on normal equations.
fn solve_normal(ata: &mut [f64], atb: &mut [f64], n: usize) -> Vec<f64> {
    for col in 0..n {
        let piv = (col..n)
            .max_by(|a, b| {
                ata[a * n + col]
                    .abs()
                    .partial_cmp(&ata[b * n + col].abs())
                    .unwrap()
            })
            .unwrap();
        if piv != col {
            for j in 0..n {
                ata.swap(col * n + j, piv * n + j);
            }
            atb.swap(col, piv);
        }
        let d = ata[col * n + col];
        for r in (col + 1)..n {
            let f = ata[r * n + col] / d;
            for j in col..n {
                ata[r * n + j] -= f * ata[col * n + j];
            }
            atb[r] -= f * atb[col];
        }
    }
    let mut x = vec![0.0f64; n];
    for r in (0..n).rev() {
        let mut s = atb[r];
        for j in (r + 1)..n {
            s -= ata[r * n + j] * x[j];
        }
        x[r] = s / ata[r * n + r];
    }
    x
}

/// Least-squares fit over the coast of
///
/// ```text
///   baro(t) = z0 + v0*tau + Z(tau)  [+ 0.5*b*tau^2]  [+ c*airspeed^2]
/// ```
///
/// The accelerometer supplies `Z` (its double integral); `z0` and `v0` are
/// the unknown coast entry state, `b` an optional accelerometer-bias term,
/// and `c` the static-port coefficient. Nothing here assumes a value for
/// `c` or an apogee anchor — the fit says what the flight data alone
/// supports, and comparing the `with_bias` variants shows how much of `c`
/// is really separable from plain inertial drift.
fn fit_port_model(
    rows: &[Measurement],
    basis: &CoastBasis,
    with_bias: bool,
    with_c: bool,
) -> (f32, f32, f32, f32, f32) {
    let n_par = 2 + with_bias as usize + with_c as usize;
    let mut ata = vec![0.0f64; n_par * n_par];
    let mut atb = vec![0.0f64; n_par];
    let mut rows_used = 0usize;

    let regressors = |k: usize| -> [f64; 4] {
        let tau = basis.tau[k] as f64;
        let vv = basis.v_int[k] * basis.tilt_gain(k);
        let mut r = [1.0, tau, 0.0, 0.0];
        let mut next = 2;
        if with_bias {
            r[next] = 0.5 * tau * tau;
            next += 1;
        }
        if with_c {
            r[next] = (vv * vv) as f64;
        }
        r
    };

    for k in 0..basis.tau.len() {
        let i = basis.from_i + k;
        if i > basis.apogee_i {
            break;
        }
        let r = regressors(k);
        let y = (rows[i].altitude_asl() - basis.z_int[k]) as f64;
        for a in 0..n_par {
            for b in 0..n_par {
                ata[a * n_par + b] += r[a] * r[b];
            }
            atb[a] += r[a] * y;
        }
        rows_used += 1;
    }

    let x = solve_normal(&mut ata, &mut atb, n_par);

    // Residual RMS of the fitted model against the raw baro.
    let mut sse = 0.0f64;
    for k in 0..basis.tau.len() {
        let i = basis.from_i + k;
        if i > basis.apogee_i {
            break;
        }
        let r = regressors(k);
        let pred: f64 = (0..n_par).map(|a| r[a] * x[a]).sum();
        let y = (rows[i].altitude_asl() - basis.z_int[k]) as f64;
        sse += (y - pred) * (y - pred);
    }
    let rms = (sse / rows_used as f64).sqrt() as f32;
    let c = if with_c { x[n_par - 1] as f32 } else { 0.0 };

    // The physics check that no amount of curve fitting can dodge: at
    // apogee the true vertical velocity is exactly 0. Each fit implies a
    // velocity trajectory v(tau) = v0 + V(tau) [+ b*tau], so evaluating it
    // at apogee scores the fit against a fact neither sensor supplied.
    let k_a = basis.apogee_i - basis.from_i;
    let b = if with_bias { x[2] as f32 } else { 0.0 };
    let v_apogee = x[1] as f32 + basis.v_int[k_a] + b * basis.tau[k_a];
    (c, rms, x[1] as f32, b, v_apogee)
}

/// Diagnostic (run with --ignored --nocapture): what do the two flight
/// logs actually say about the static-port coefficient, and is it
/// separable from ordinary inertial drift? Fits the coast four ways —
/// with/without a `c` term, with/without an accelerometer-bias term — and
/// prints the fitted `c` and the residual RMS of each.
#[test]
#[ignore]
fn port_coefficient_fit() {
    init_logger();

    for (name, rows, honest_after_ignition_us) in [
        // Void Lake is subsonic; skip the ±16 g clipped boost.
        ("Void Lake (subsonic)", void_lake_rows(), 4_000_000u64),
        // LC'25 goes Mach 2: only fit where the baro is subsonic-honest
        // (below Mach 0.8 from ~13 s after ignition detection).
        ("LC'25 (Mach 2)", lc25_rows(), 13_000_000),
    ] {
        let ign_i = find_ignition(&rows);
        let apogee_i = true_apogee(&rows);
        let from = rows
            .iter()
            .position(|r| r.timestamp_us > rows[ign_i].timestamp_us + honest_after_ignition_us)
            .unwrap();
        let mut basis = coast_basis(&rows, from, apogee_i);
        eprintln!(
            "=== {name}: coast fit t={:.1}..{:.1}s (true apogee; `baro_apogee` says {:.1}s)",
            t_s(&rows, from),
            t_s(&rows, apogee_i),
            t_s(&rows, baro_apogee(&rows).0),
        );
        for (label, with_bias, with_c) in [
            ("no c, no bias", false, false),
            ("c only", false, true),
            ("bias only", true, false),
            ("c + bias", true, true),
        ] {
            let (c, rms, v0, b, v_apogee) = fit_port_model(&rows, &basis, with_bias, with_c);
            eprintln!(
                "    {label:<14} c = {c:>9.2e}  v0 = {v0:>6.1} m/s  b = {b:>6.3} m/s2  \
                 RMS {rms:>6.2} m  vv(apogee) = {v_apogee:>+6.1} m/s"
            );
        }

        // The sweep that needs no collinearity argument: hold c fixed,
        // fit only the coast entry state, and see which c the baro
        // actually prefers. Run it with the plain vertical velocity and
        // again with the estimator's own tilt-corrected airspeed, so a
        // drifted tilt cannot be blamed for the answer.
        for use_tilt in [false, true] {
            basis.use_tilt = use_tilt;
            eprintln!(
                "    -- fixed-c residual sweep ({}) --",
                if use_tilt { "airspeed = vv/cos(tilt)" } else { "airspeed = vv" }
            );
            eprintln!(
                "    {:>9} {:>16} {:>16} {:>14}",
                "c", "RMS (z0,v0)", "RMS (z0,v0,bias)", "vv(apogee)"
            );
            for c in [0.0, 0.25e-3, 0.5e-3, 0.7e-3, 1.5e-3, 2.5e-3] {
                let (rms_a, v_a) = fixed_c_residual(&rows, &basis, c, false);
                let (rms_b, _) = fixed_c_residual(&rows, &basis, c, true);
                eprintln!("    {c:>9.2e} {rms_a:>13.2} m {rms_b:>14.2} m {v_a:>+11.1} m/s");
            }
        }
    }
}

/// Diagnostic (run with --ignored --nocapture): when does each candidate
/// vote first call Mach 0.8, and how wrong can it be?
///
/// Today votes 1 and 3 BOTH read `reckoner.velocity` — vote 1 compares its
/// magnitude to Mach 0.8, vote 3 compares the baro's slope to its vertical
/// component. One drifting dead reckoner therefore moves two of three
/// votes together, and 2-of-3 stops being protection.
///
/// The drag vote breaks that tie: in free flight the accelerometer's raw
/// magnitude IS drag/mass, so `q = a*m/(Cd*A)` inverts to a speed with no
/// integration, no attitude and no baro slope. This prints it against the
/// dead-reckoned Mach, and sweeps the one constant it needs (Cd*A/m) to
/// see how much a wrong drag model costs.
#[test]
#[ignore]
fn drag_vote_timing_and_sensitivity() {
    use super::dead_reckoner::DeadReckoner;
    use crate::utils::{approximate_air_density, approximate_speed_of_sound};
    init_logger();
    let up = Vector3::new(0.0f32, 0.0, 1.0);
    // Cd*A/m for this airframe. In flight this comes from ROCKET_PARAMETERS,
    // which the MPC already requires — it is not a new unknown. Using the
    // SUBSONIC Cd is deliberate: the true Cd is higher transonically, so
    // the inverted speed reads HIGH exactly while supersonic, which is the
    // conservative direction for a "have we slowed down yet" gate.
    const K_SUBSONIC: f32 = 0.00024;

    let clean = lc25_rows();

    let mach_tracks = |rows: &[Measurement], k: f32| -> Vec<(f32, f32, f32, f32, f32, f32)> {
        let ign_i = find_ignition(rows);
        let t_ign = rows[ign_i].timestamp_us;
        let (mut acc_sum, mut gyro_sum, mut n) =
            (Vector3::<f32>::zeros(), Vector3::<f32>::zeros(), 0usize);
        for r in rows {
            let back = t_ign.saturating_sub(r.timestamp_us);
            if (200_000..=2_200_000).contains(&back) {
                acc_sum += r.acceleration();
                gyro_sum += r.angular_velocity();
                n += 1;
            }
        }
        let gravity = acc_sum / n as f32;
        let bias = gyro_sum / n as f32;
        let mut dr = DeadReckoner::new(q_from_vecs(&up, &gravity));
        dr.position.z = rows[ign_i].altitude_asl();
        let mut prev_t = rows[0].timestamp_us;
        let mut out = Vec::new();
        // The drag channel is a single raw sample, so it carries the full
        // accelerometer noise and airframe vibration. Low-pass it the same
        // way ignition detection low-passes its own channel, otherwise one
        // noisy sample trips the vote a second early.
        let mut lp: Option<f32> = None;
        const TAU_S: f32 = 0.3;
        for z in rows.iter() {
            let dt = ((z.timestamp_us.saturating_sub(prev_t)) as f32 * 1e-6).clamp(0.0, MAX_DT_S);
            prev_t = z.timestamp_us;
            dr.update(&z.acceleration(), &(z.angular_velocity() - bias), dt);
            let sos = approximate_speed_of_sound(z.altitude_asl());
            let rho = approximate_air_density(z.altitude_asl());
            let a_raw = z.acceleration().magnitude();
            let alpha = (dt / TAU_S).min(1.0);
            let a = match lp {
                Some(prev) => prev + alpha * (a_raw - prev),
                None => a_raw,
            };
            lp = Some(a);
            // rho from the BARO altitude — which is exactly what is lying
            // during the supersonic phase this vote has to survive...
            let v_drag = (2.0 * a / (rho * k)).sqrt();
            // ...versus rho from the DEAD-RECKONED altitude, which makes
            // the drag vote completely baro-free.
            let rho_dr = approximate_air_density(dr.position.z);
            let sos_dr = approximate_speed_of_sound(dr.position.z);
            let v_drag_dr = (2.0 * a / (rho_dr * k)).sqrt();
            out.push((
                (z.timestamp_us.saturating_sub(t_ign)) as f32 * 1e-6,
                dr.velocity.magnitude() / sos,
                v_drag / sos,
                v_drag_dr / sos_dr,
                z.altitude_asl(),
                dr.position.z,
            ));
        }
        out
    };

    let track = mach_tracks(&clean, K_SUBSONIC);
    eprintln!("=== LC'25: dead-reckoned Mach vs drag-inverted Mach (Cd*A/m = {K_SUBSONIC:.5})");
    eprintln!(
        "    {:>7} {:>8} {:>9} {:>10} {:>10} {:>9} {:>8}",
        "t-ign", "DR M", "drag M", "drag M/DR", "baro alt", "DR alt", "baro-DR"
    );
    let mut next = 5.0f32;
    for (t, dr_m, drag_m, drag_m_dr, baro_alt, dr_alt) in &track {
        if *t < next || *t > 16.0 {
            continue;
        }
        next = t + 0.5;
        eprintln!(
            "    {t:>+7.1} {dr_m:>8.2} {drag_m:>9.2} {drag_m_dr:>10.2} {baro_alt:>10.0} {dr_alt:>9.0} {:>8.0}",
            baro_alt - dr_alt
        );
    }

    // The drag vote needs Cd*A/m. It is not a new unknown — burnout_mass,
    // cd[0] and reference_area are already in ROCKET_PARAMETERS because
    // the MPC needs them — but it is worth knowing what an error costs.
    // v scales as 1/sqrt(k), so k too HIGH reads the speed LOW and calls
    // subsonic early: that is the unsafe direction.
    eprintln!("    -- sensitivity to Cd*A/m (first time each vote drops below Mach 0.8) --");
    eprintln!("    {:>12} {:>10} {:>16}", "Cd*A/m", "error", "drag M < 0.8 at");
    type MachRow = (f32, f32, f32, f32, f32, f32);
    let first_below = |track: &[MachRow], pick: fn(&MachRow) -> f32| {
        track
            .iter()
            // only meaningful in free flight; LC'25 burns out at +6.1 s and
            // the lockout's own T_min is +8 s
            .find(|r| r.0 > 6.5 && pick(r) < 0.8)
            .map(|r| r.0)
    };
    for scale in [0.7f32, 0.85, 1.0, 1.15, 1.3] {
        let t = mach_tracks(&clean, K_SUBSONIC * scale);
        eprintln!(
            "    {:>12.5} {:>9.0}% {:>14.1?}s",
            K_SUBSONIC * scale,
            (scale - 1.0) * 100.0,
            first_below(&t, |r| r.2)
        );
    }
    eprintln!(
        "    for reference, the inertial vote (DR) drops below Mach 0.8 at {:.1?}s",
        first_below(&track, |r| r.1)
    );

    // The burnout tail-off dip: residual thrust cancels part of the drag,
    // so |accel| collapses and the drag vote reads FALSE SUBSONIC while
    // still genuinely supersonic. This is the drag vote's own unsafe
    // failure, and the reason it cannot stand alone without T_min.
    let burn_i = find_burnout(&clean);
    let ign_i = find_ignition(&clean);
    eprintln!(
        "    burnout at ignition+{:.1}s; drag M around it:",
        t_s(&clean, burn_i) - t_s(&clean, ign_i)
    );
    for (t, dr_m, drag_m, ..) in &track {
        if *t > 5.0 && *t < 7.5 {
            let tag = if drag_m < &0.8 { "   <-- FALSE SUBSONIC" } else { "" };
            eprintln!("      +{t:.2}  DR M {dr_m:.2}  drag M {drag_m:.2}{tag}");
        }
    }
}

/// Burnout index: first sample after ignition whose 0.2 s rolling mean of
/// |acc| falls below 1.5 g. After this the airframe is in free flight, so
/// the accelerometer's raw magnitude IS drag / mass.
fn find_burnout(rows: &[Measurement]) -> usize {
    let ign_i = find_ignition(rows);
    let t_ign = rows[ign_i].timestamp_us;
    let (mut lo, mut sum, mut cnt) = (ign_i, 0.0f32, 0usize);
    for i in ign_i..rows.len() {
        sum += rows[i].acceleration().magnitude();
        cnt += 1;
        while rows[i].timestamp_us - rows[lo].timestamp_us > 200_000 {
            sum -= rows[lo].acceleration().magnitude();
            cnt -= 1;
            lo += 1;
        }
        if rows[i].timestamp_us > t_ign + 1_000_000 && cnt > 10 && (sum / cnt as f32) < 1.5 * 9.81 {
            return i;
        }
    }
    panic!("no burnout found");
}

/// Diagnostic (run with --ignored --nocapture): the evidence behind the
/// drag vote. In free flight `|acc|` is drag/mass, so dividing it by the
/// dynamic pressure implied by the dead-reckoned speed recovers `Cd*A/m`.
/// If the drag model holds, that column is CONSTANT subsonically — and on
/// LC'25 it is (0.00022-0.00026), landing on the same value
/// `ROCKET_PARAMETERS` gives analytically. It rises ~40% through the
/// transonic peak, which is exactly why inverting with the subsonic Cd
/// makes the vote read high while supersonic.
#[test]
#[ignore]
fn mach_detection_signals() {
    use super::dead_reckoner::DeadReckoner;
    use crate::utils::{approximate_air_density, approximate_speed_of_sound};
    init_logger();
    let up = Vector3::new(0.0f32, 0.0, 1.0);

    for (name, rows) in [
        ("LC'25 (Mach 2)", lc25_rows()),
        ("Void Lake (subsonic)", void_lake_rows()),
    ] {
        let ign_i = find_ignition(&rows);
        let burn_i = find_burnout(&rows);
        let apogee_i = true_apogee(&rows);
        let t_ign = rows[ign_i].timestamp_us;

        let (mut acc_sum, mut gyro_sum, mut n) =
            (Vector3::<f32>::zeros(), Vector3::<f32>::zeros(), 0usize);
        for r in &rows {
            let back = t_ign.saturating_sub(r.timestamp_us);
            if (200_000..=2_200_000).contains(&back) {
                acc_sum += r.acceleration();
                gyro_sum += r.angular_velocity();
                n += 1;
            }
        }
        let gravity = acc_sum / n as f32;
        let bias = gyro_sum / n as f32;
        let mut dr = DeadReckoner::new(q_from_vecs(&up, &gravity));
        dr.position.z = rows[ign_i].altitude_asl();

        eprintln!(
            "=== {name}: ignition t={:.1}s, burnout +{:.1}s, apogee +{:.1}s",
            t_s(&rows, ign_i),
            t_s(&rows, burn_i) - t_s(&rows, ign_i),
            t_s(&rows, apogee_i) - t_s(&rows, ign_i),
        );
        eprintln!(
            "    {:>7} {:>9} {:>7} {:>9} {:>10} {:>10}",
            "t-ign", "DR speed", "DR M", "drag a", "q (kPa)", "CdA/m"
        );

        let mut prev_t = rows[0].timestamp_us;
        let mut next_print = t_ign;
        for (i, z) in rows.iter().enumerate() {
            let dt = ((z.timestamp_us.saturating_sub(prev_t)) as f32 * 1e-6).clamp(0.0, MAX_DT_S);
            prev_t = z.timestamp_us;
            dr.update(&z.acceleration(), &(z.angular_velocity() - bias), dt);
            if i > apogee_i {
                break;
            }
            if z.timestamp_us < next_print {
                continue;
            }
            next_print = z.timestamp_us + 500_000;

            let speed = dr.velocity.magnitude();
            let sos = approximate_speed_of_sound(z.altitude_asl());
            let rho = approximate_air_density(z.altitude_asl());
            let drag_a = z.acceleration().magnitude();
            let q = 0.5 * rho * speed * speed;
            eprintln!(
                "    {:>+7.1} {:>9.1} {:>7.2} {:>9.2} {:>10.1} {:>10.5}{}",
                t_s(&rows, i) - t_s(&rows, ign_i),
                speed,
                speed / sos,
                drag_a,
                q * 1e-3,
                if q > 100.0 { drag_a / q } else { f32::NAN },
                if i < burn_i { "  (thrusting)" } else { "" },
            );
        }
    }
}

/// Diagnostic (run with --ignored --nocapture): can the accelerometer
/// detect burnout on its own, so the airbrakes half stops needing the baro
/// half's `is_coasting()` timer?
///
/// The discriminator is the SIGN of the axial specific force, not its
/// magnitude. Thrust acts along +axis, drag along -axis, so the axial
/// channel crosses zero at burnout and stays negative for the whole coast.
/// A magnitude test cannot see that crossing; a sign test can.
///
/// Prints the axial channel through burnout for both logs, plus how long
/// the zero crossing takes (the one window where a burning motor could be
/// mistaken for free flight).
#[test]
#[ignore]
fn axial_sign_detects_burnout() {
    init_logger();
    for (name, rows) in [
        ("LC'25 (Mach 2)", lc25_rows()),
        ("Void Lake (subsonic)", void_lake_rows()),
    ] {
        let ign_i = find_ignition(&rows);
        let burn_i = find_burnout(&rows);
        let t_ign = rows[ign_i].timestamp_us;

        // Rocket axis in the body frame: gravity on the pad, which points
        // along the airframe axis while it sits on the rail.
        let (mut acc_sum, mut n) = (Vector3::<f32>::zeros(), 0usize);
        for r in &rows {
            let back = t_ign.saturating_sub(r.timestamp_us);
            if (200_000..=2_200_000).contains(&back) {
                acc_sum += r.acceleration();
                n += 1;
            }
        }
        let axis = (acc_sum / n as f32).normalize();

        let axial = |i: usize| rows[i].acceleration().dot(&axis);

        // First sample after ignition where the axial channel goes negative
        // and STAYS negative for 0.3 s — a candidate burnout latch.
        let mut latch = None;
        let mut neg_since: Option<u64> = None;
        for i in ign_i..rows.len() {
            if axial(i) < -2.0 {
                let t0 = *neg_since.get_or_insert(rows[i].timestamp_us);
                if rows[i].timestamp_us - t0 >= 300_000 {
                    latch = Some(i);
                    break;
                }
            } else {
                neg_since = None;
            }
        }

        eprintln!(
            "=== {name}: |acc|-based burnout at ignition+{:.2}s, axial-sign latch at ignition+{:.2}s",
            t_s(&rows, burn_i) - t_s(&rows, ign_i),
            latch.map(|i| t_s(&rows, i) - t_s(&rows, ign_i)).unwrap_or(f32::NAN),
        );
        eprintln!("    {:>8} {:>10} {:>10}", "t-ign", "axial", "|acc|");
        let mut next = t_ign;
        for i in ign_i..rows.len() {
            let rel = (rows[i].timestamp_us - t_ign) as f32 * 1e-6;
            if rel > 12.0 {
                break;
            }
            if rows[i].timestamp_us < next {
                continue;
            }
            next = rows[i].timestamp_us + 250_000;
            eprintln!(
                "    {:>+8.2} {:>10.2} {:>10.2}{}",
                rel,
                axial(i),
                rows[i].acceleration().magnitude(),
                if Some(i) == latch { "   <- burnout latched" } else { "" },
            );
        }
    }
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
