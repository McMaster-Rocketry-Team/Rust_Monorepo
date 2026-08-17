use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::Vector3;

use super::*;
use crate::{
    FlightConfig, FlightEstimators, ImuSample,
    tests::fixtures::{lc25_airbrakes, subsonic_profile},
    tests::init_logger,
};

/// One row of a replayed log: exactly the three arguments
/// [`AirbrakesEstimator::update`] takes, kept together so a whole flight can
/// live in a `Vec` and be indexed by sample.
struct Row {
    timestamp_us: u64,
    imu: ImuSample,
    altitude_asl: f32,
}

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
fn void_lake_rows() -> Vec<Row> {
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
            Row {
                timestamp_us: r.timestamp_us,
                imu: ImuSample {
                    acc: Vector3::new(r.acc_x, r.acc_y, r.acc_z),
                    // VLF5 logs gyro in deg/s
                    gyro: Vector3::new(
                        r.gyro_x.to_radians(),
                        r.gyro_y.to_radians(),
                        r.gyro_z.to_radians(),
                    ),
                },
                altitude_asl: calculate_isa_altitude(Pascals(r.pressure as f64)).0 as f32,
            }
        })
        .collect()
}

fn lc25_rows() -> Vec<Row> {
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
            Row {
                timestamp_us: r.time_us,
                imu: ImuSample {
                    // this recorder's acc y/z are sign-flipped relative to
                    // its gyro frame
                    acc: Vector3::new(r.imu_acc_x, -r.imu_acc_y, -r.imu_acc_z),
                    gyro: Vector3::new(
                        r.gyro_x.to_radians(),
                        r.gyro_y.to_radians(),
                        r.gyro_z.to_radians(),
                    ),
                },
                altitude_asl: r.altitude,
            }
        })
        .collect()
}

/// Loop the log's own pad segment (everything up to 0.3 s before ignition,
/// so no spool-up gets copied) in front of the log until at least
/// `extra_s` more pad exists. The copies carry the pad's real noise, sway
/// and timestamp jitter — only the wall-clock length of the rail wait is
/// synthetic. Everything from the original log keeps its raw relative
/// timing (the whole log is just shifted later by the added span).
fn extend_pad(rows: Vec<Row>, extra_s: f32) -> Vec<Row> {
    let ign_i = find_ignition(&rows);
    let t_ign = rows[ign_i].timestamp_us;
    let pad: Vec<&Row> = rows
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
            out.push(Row {
                timestamp_us: r.timestamp_us + k * period,
                imu: r.imu.clone(),
                altitude_asl: r.altitude_asl,
            });
        }
    }
    for r in &rows {
        out.push(Row {
            timestamp_us: r.timestamp_us + copies * period,
            imu: r.imu.clone(),
            altitude_asl: r.altitude_asl,
        });
    }
    out
}

/// Baro-derived vertical velocity reference: central difference over
/// +-0.5 s of real timestamps. Honest where the baro is honest (subsonic,
/// low dynamic pressure).
fn reference_velocity(rows: &[Row], i: usize) -> f32 {
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
    (rows[hi].altitude_asl - rows[lo].altitude_asl) / dt
}

/// (index, altitude) of the highest 1 s-smoothed baro altitude.
fn baro_apogee(rows: &[Row]) -> (usize, f32) {
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
            sum += rows[j].altitude_asl;
            n += 1;
        }
        let avg = sum / n as f32;
        if avg > best.1 {
            best = (i, avg);
        }
    }
    best
}

fn t_s(rows: &[Row], i: usize) -> f32 {
    (rows[i].timestamp_us - rows[0].timestamp_us) as f32 * 1e-6
}

struct ReplayResult {
    birth: Option<(u64, bool)>,
    /// (row index, estimated altitude ASL) of the estimator's apogee call —
    /// the first sample its vertical velocity reached zero. See `replay`.
    apogee_i: Option<usize>,
    apogee_alt_asl: Option<f32>,
    /// (row index, estimated vv) while the filter was alive
    vv_track: Vec<(usize, f32)>,
    /// (row index, estimated altitude ASL) for every sample the estimator
    /// reported one — i.e. from ignition on, filter or dead reckoning.
    alt_track: Vec<(usize, f32)>,
    /// continuous spans (start s, end s) where the drag check held true
    subsonic_spans: Vec<(f32, f32)>,
    /// wall time (s from log start) when `burnout_detected()` first went true
    burnout_s: Option<f32>,
    /// set if `burnout_detected()` ever went back to false after latching
    burnout_unlatched: bool,
    /// wall time (s from log start) when pad calibration first completed
    calibration_complete_s: Option<f32>,
}

fn replay(rows: &[Row], config: AirbrakesConfig) -> ReplayResult {
    let mut estimator = AirbrakesEstimator::new(config);
    let mut result = ReplayResult {
        birth: None,
        apogee_i: None,
        apogee_alt_asl: None,
        vv_track: Vec::new(),
        alt_track: Vec::new(),
        subsonic_spans: Vec::new(),
        burnout_s: None,
        burnout_unlatched: false,
        calibration_complete_s: None,
    };
    let mut subsonic_span_start: Option<f32> = None;
    for (i, z) in rows.iter().enumerate() {
        estimator.update(z.timestamp_us, &z.imu, z.altitude_asl);

        let now = t_s(rows, i);
        match (estimator.burnout_detected(), result.burnout_s) {
            (true, None) => result.burnout_s = Some(now),
            (false, Some(_)) => result.burnout_unlatched = true,
            _ => {}
        }
        if result.calibration_complete_s.is_none() && estimator.calibration_complete() {
            result.calibration_complete_s = Some(now);
        }
        match (estimator.subsonic_by_drag(), subsonic_span_start) {
            (Some(true), None) => subsonic_span_start = Some(now),
            (Some(true), Some(_)) => {}
            (_, Some(start)) => {
                result.subsonic_spans.push((start, now));
                subsonic_span_start = None;
            }
            _ => {}
        }
        if result.birth.is_none() {
            result.birth = estimator.birth();
        }
        if let Some(v) = estimator.velocity() {
            result.vv_track.push((i, v.y));
        }
        if let Some(a) = estimator.altitude_asl() {
            result.alt_track.push((i, a));
        }
        // Apogee, on a BARE estimator: the first sample the vertical filter
        // reports a non-positive vertical velocity.
        //
        // This is not a stand-in for something the estimator does internally
        // — it IS the criterion the flight uses. `FlightEstimators::update`
        // retires the airbrakes half on `velocity().y <= 0`, and nothing else
        // in the system calls apogee for this filter. The estimator has no
        // apogee state to ask (the 0.5 s / 1 m/s latch that used to provide
        // one was deleted on 2026-08-17: it lost this race by 0.389 s on Void
        // Lake and 0.392 s on LC'25, and could not have won it anyway — the
        // trajectory spends only 0.108 s / 0.106 s below 1 m/s).
        //
        // A bare estimator is not retired, so unlike the real flight the
        // filter keeps running past this instant; the tests below score only
        // the first crossing.
        if result.apogee_i.is_none()
            && let Some(v) = estimator.velocity()
            && v.y <= 0.0
        {
            result.apogee_i = Some(i);
            result.apogee_alt_asl = estimator.altitude_asl();
        }
    }
    result
}

/// Mean |vv error| vs the baro-rate reference over rows [from, to).
fn vv_error(rows: &[Row], track: &[(usize, f32)], from: usize, to: usize) -> (f32, usize) {
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

    // Subsonic profile (no Mach lockout, which is what `lc25_airbrakes`
    // is): the drag check is never consulted, so the airframe cannot
    // affect this run.
    let result = replay(&rows, lc25_airbrakes());

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

/// The property the burnout latch exists for, on BOTH profiles: the
/// vertical filter — and therefore the MPC state, and therefore any chance
/// of the brakes opening — must never come alive while the motor is
/// burning.
///
/// This is the airbrakes half's own guarantee: both paths are gated on the
/// measured axial-sign latch, and the gate needs no cross-half input.
#[test]
fn filter_is_never_born_under_thrust() {
    init_logger();
    for (name, rows, config) in [
        (
            "Void Lake (subsonic profile)",
            extend_pad(void_lake_rows(), 8.0),
            lc25_airbrakes(),
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

/// LC'25 as a Mach 2 flight: the same airframe, threshold and
/// `max_open_mach` as [`lc25_airbrakes`], plus the lockout window that
/// makes it supersonic.
///
/// The two timers are the only thing overridden, and they are overridden
/// here rather than defaulted in the fixture because they are what several
/// of the tests below actually measure — the drag check has to fire inside
/// this window, and `forced_birth_backstop_flies_the_rest_of_the_flight`
/// deliberately drives it to the far edge.
fn lc25_config() -> AirbrakesConfig {
    AirbrakesConfig {
        // (times from ignition detection) true 0.75 M crossing ~12.6 s,
        // 0.8 M slightly earlier: T_min well before, T_max bounded well
        // before apogee (~32 s after ignition)
        mach_lockout: Some(MachLockoutConfig {
            earliest_subsonic_after_ignition_us: 8_000_000,
            force_birth_after_ignition_us: 20_000_000,
        }),
        ..lc25_airbrakes()
    }
}

/// LC'25, RAW 500 Hz timestamps, Mach 2 profile: the drag check (not the timer)
/// must birth the filter, at an honest time — and its flip times form
/// the truth table the plan requires. The recorder started only ~1.8 s
/// before ignition, so the pad is extended with its own looped noise to
/// let calibration complete (see `extend_pad`); all check/birth times below
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
        "lc25 v2: born at ignition+{birth_rel:.1}s (forced: {forced}), drag-check spans (rel) {:?}",
        result
            .subsonic_spans
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

    // The property that makes a single measurement sufficient: the
    // drag-inverted speed must NEVER read subsonic while the airframe is
    // genuinely supersonic (before ignition+11 s). Not "not for long" —
    // never, and momentary flickers are not tolerated either.
    for (start, end) in &result.subsonic_spans {
        assert!(
            start - ign_s >= 11.0,
            "drag check read subsonic at ignition+{}s..{}s, while still supersonic",
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
/// Lake failure, injected into the Mach 2 flight): the inertial estimate
/// reads low and early, but that must NOT open the lockout — and after
/// birth the baro must pull the wrong dead-reckoned velocity back.
#[test]
fn lc25_clipped_accel_replay() {
    init_logger();
    let mut rows = extend_pad(lc25_rows(), 12.0);
    const RAIL: f32 = 16.0 * 9.81;
    for r in &mut rows {
        r.imu.acc = r.imu.acc.map(|a| a.clamp(-RAIL, RAIL));
    }
    let (apogee_ref_i, _) = baro_apogee(&rows);
    let ign_s = t_s(&rows, find_ignition(&rows));
    let result = replay(&rows, lc25_config());

    let (birth_t, _forced) = result.birth.expect("filter never born");
    let birth_rel = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;
    eprintln!("lc25 clipped: born at ignition+{birth_rel:.1}s");

    // The under-reading inertial estimate must not exit while genuinely
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
/// on pad through the entire flight and says so.
///
/// The windows it keeps closing DURING that flight are the whole point:
/// they must not fake a calibration. This is the only test that fails if
/// `screen_pad_windows` is reduced to a plain average, and it is what the
/// absolute 1 g / no-rotation check exists for — in-flight windows do not
/// look like a pad, which is a stronger reason to drop them than the old
/// one (that they disagreed with each other). In the real
/// system arming is blocked on `calibration_complete()`, so this flight
/// would never have left the rail.
#[test]
fn short_pad_refuses_ignition() {
    init_logger();
    let rows = lc25_rows();
    let mut estimator = AirbrakesEstimator::new(lc25_config());
    for (i, z) in rows.iter().enumerate() {
        estimator.update(z.timestamp_us, &z.imu, z.altitude_asl);
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
    assert!(estimator.velocity().is_none(), "handed out an MPC velocity");
}

/// A knock on the rail must not latch ignition — the airbrakes half's
/// version of `baro_state_estimator::tests::a_knock_on_the_pad_does_not_latch_ignition`,
/// and the regression guard for the defect that motivated sharing the
/// detector: until 2026-08-17 this half had a threshold and a low pass but
/// no sustain, so it latched on the FIRST sample over threshold. A 10 ms
/// knock was enough, and the cost was not a spurious log line — ignition
/// detection is the origin of the Mach lockout clock, so a pad latch means
/// the lockout timers expire against the wrong zero.
///
/// The first phase exists to open the calibration gate. Without it this
/// test proves nothing: `State::OnPad` refuses to detect ignition before
/// calibration completes, so knocking at a rocket that has not calibrated
/// yet is caught by the wrong mechanism. The assertion that calibration IS
/// complete while the knocking happens is what makes the sustain the only
/// thing left holding the latch shut.
#[test]
fn a_knock_on_the_pad_does_not_latch_ignition() {
    init_logger();
    // This test's OWN synthetic sample spacing, not something the estimator
    // knows: it feeds a perfectly regular 416 Hz rail because a knock is
    // easier to place in time that way. The estimator reads only the
    // timestamps it is handed.
    const KNOCK_DT_S: f32 = 1.0 / 416.0;
    let dt_us = (KNOCK_DT_S * 1e6) as u64;
    let mut estimator = AirbrakesEstimator::new(lc25_config());
    let gyro = Vector3::zeros();

    // Phase 1: 10 s of quiet rail, enough for >= 3 screened 2 s windows.
    let mut t_us = 0u64;
    for _ in 0..(10.0 / KNOCK_DT_S) as u64 {
        let imu = ImuSample {
            acc: Vector3::new(0.0, 0.0, 9.81),
            gyro,
        };
        estimator.update(t_us, &imu, 200.0);
        t_us += dt_us;
    }
    assert!(
        estimator.calibration_complete(),
        "10 s of quiet pad did not produce a calibration — the knock phase \
         below would be gated by the wrong thing"
    );

    // Phase 2: 10 s of the same rail with a 40 ms 12 g knock every second.
    // Mirror the shared detector's low pass alongside, to prove the
    // transient really does cross the threshold.
    let mut lp: Option<Vector3<f32>> = None;
    let mut peak_lp = 0.0f32;
    let threshold = lc25_config().ignition_detection_acc_threshold;
    for k in 0..(10.0 / KNOCK_DT_S) as u64 {
        let t = k as f32 * KNOCK_DT_S;
        let acc = if (t % 1.0) < 0.04 {
            Vector3::new(0.0, 0.0, 12.0 * 9.81)
        } else {
            Vector3::new(0.0, 0.0, 9.81)
        };
        estimator.update(t_us, &ImuSample { acc, gyro }, 200.0);
        t_us += dt_us;

        // 0.0159 s is `ignition_detector::LP_TAU_S`.
        let v = match lp {
            Some(prev) => prev + (KNOCK_DT_S / 0.0159f32).min(1.0) * (acc - prev),
            None => acc,
        };
        lp = Some(v);
        peak_lp = peak_lp.max(v.magnitude());

        assert!(
            estimator.launch_pad_altitude_asl().is_none(),
            "a knock at t={t:.3}s latched ignition and left the pad state"
        );
        assert!(estimator.birth().is_none(), "a knock at t={t:.3}s birthed the filter");
    }

    eprintln!(
        "airbrakes knock test: worst low-passed |acc| reached {:.1} m/s^2 = \
         {:.1} g (threshold {:.1} g), calibration complete, nothing latched",
        peak_lp,
        peak_lp / 9.81,
        threshold / 9.81
    );
    assert!(
        peak_lp > threshold,
        "the knock never crossed the threshold ({:.1} m/s^2 vs {:.1}) — this \
         test proves nothing",
        peak_lp,
        threshold
    );
}

/// Burnout index: first sample after ignition whose 0.2 s rolling mean of
/// |acc| falls below 1.5 g. After this the airframe is in free flight, so
/// the accelerometer's raw magnitude IS drag / mass.
fn find_burnout(rows: &[Row]) -> usize {
    let ign_i = find_ignition(rows);
    let t_ign = rows[ign_i].timestamp_us;
    let (mut lo, mut sum, mut cnt) = (ign_i, 0.0f32, 0usize);
    for i in ign_i..rows.len() {
        sum += rows[i].imu.acc.magnitude();
        cnt += 1;
        while rows[i].timestamp_us - rows[lo].timestamp_us > 200_000 {
            sum -= rows[lo].imu.acc.magnitude();
            cnt -= 1;
            lo += 1;
        }
        if rows[i].timestamp_us > t_ign + 1_000_000 && cnt > 10 && (sum / cnt as f32) < 1.5 * 9.81 {
            return i;
        }
    }
    panic!("no burnout found");
}

/// Ignition index: first sample whose 0.1 s rolling mean of |acc| exceeds
/// 4 g.
fn find_ignition(rows: &[Row]) -> usize {
    let (mut lo, mut sum, mut cnt) = (0usize, 0.0f32, 0usize);
    for i in 0..rows.len() {
        sum += rows[i].imu.acc.magnitude();
        cnt += 1;
        while rows[i].timestamp_us - rows[lo].timestamp_us > 100_000 {
            sum -= rows[lo].imu.acc.magnitude();
            cnt -= 1;
            lo += 1;
        }
        if cnt > 10 && sum / cnt as f32 > 4.0 * 9.81 {
            return i;
        }
    }
    panic!("no ignition found");
}

/// The airbrakes half must be retired exactly once on a real flight, at
/// apogee, and never come back — the wrapper drops it on the first of
/// (vv <= 0), (tilt past horizontal), (deployment estimator at apogee).
///
/// Void Lake is the useful replay here: it flies a real ascent through a
/// real apogee, so the retirement instant can be checked against the baro
/// apogee rather than against a synthetic trajectory that would only prove
/// the `if` was typed correctly.
#[test]
fn airbrakes_half_retires_at_apogee_and_stays_retired() {
    init_logger();
    let rows = extend_pad(void_lake_rows(), 8.0);
    let (apogee_ref_i, _) = baro_apogee(&rows);

    // Nothing here reads a config number: the retirement instant is scored
    // against the baro apogee of the replayed log, not against a threshold.
    let mut est = FlightEstimators::new(FlightConfig {
        profile: subsonic_profile(),
        airbrakes: lc25_airbrakes(),
    });

    let mut retired_i: Option<usize> = None;
    let mut last_mpc_states_i: Option<usize> = None;
    for (i, z) in rows.iter().enumerate() {
        let (_pyro, log) = est.update(z.timestamp_us, Some(&z.imu), z.altitude_asl);

        // The log sample is built after retirement, so the airbrakes group
        // goes absent on the SAME sample the half is dropped — no record
        // carries airbrakes numbers from an estimator that no longer exists,
        // and none loses them a sample early.
        assert_eq!(
            log.airbrakes.is_some(),
            est.airbrakes_estimator().is_some(),
            "log sample and estimator disagree about the airbrakes half at sample {i}"
        );

        if est.airbrakes_estimator().is_some() {
            // Never resurrects: nothing may be Some after the first None.
            assert!(
                retired_i.is_none(),
                "airbrakes half came back at sample {i} after retiring at {:?}",
                retired_i
            );
        } else if retired_i.is_none() {
            retired_i = Some(i);
        }

        if est.airbrakes_mpc_states().is_some() {
            assert!(
                retired_i.is_none(),
                "MPC states handed out at sample {i} after retirement"
            );
            last_mpc_states_i = Some(i);
        }
    }

    let retired_i = retired_i.expect("airbrakes half was never retired");
    let retired_s = t_s(&rows, retired_i);
    let apogee_s = t_s(&rows, apogee_ref_i);
    eprintln!(
        "void lake: airbrakes half retired at t={retired_s:.1}s (baro apogee {apogee_s:.1}s), \
         last MPC states at {:?}",
        last_mpc_states_i.map(|i| t_s(&rows, i))
    );

    // The brakes must have been usable at some point, or this test would
    // pass on an estimator that retired itself on the pad.
    let last_mpc_states_i = last_mpc_states_i.expect("MPC states were never handed out");
    assert!(last_mpc_states_i < retired_i);

    // Retirement is an apogee event, and the two directions are not
    // equally bad. Late is the failure that matters — brakes still
    // commandable past apogee — so it gets the tight bound. Early only
    // costs brake authority, and this filter's vv is known to read low
    // near apogee on this log: `void_lake_v2_replay` allows its apogee
    // latch 3 s of error, and `vv_error` there stops 2 s short of apogee
    // for the same reason.
    assert!(
        retired_s - apogee_s < 0.5,
        "retired {:.1}s AFTER baro apogee ({retired_s:.1}s vs {apogee_s:.1}s)",
        retired_s - apogee_s
    );
    assert!(
        apogee_s - retired_s < 3.0,
        "retired {:.1}s before baro apogee ({retired_s:.1}s vs {apogee_s:.1}s)",
        apogee_s - retired_s
    );
}

// ---------------------------------------------------------------------------
// The lockout exit: the velocity ceiling it votes at, and the drag model it
// votes with.
//
// Promoted from a printing diagnostic on 2026-08-17. Everything below runs
// the REAL estimator over the raw LC'25 log and scores its birth against the
// baro-truth speed at that instant — the diagnostic it replaces re-implemented
// the drag inversion in the test and only printed the answer, so nothing in
// the suite noticed if `max_open_mach` stopped being read at all.
// ---------------------------------------------------------------------------

/// One LC'25 replay with the drag model scaled and the velocity ceiling
/// overridden, reported relative to ignition detection: `(birth s, forced)`
/// plus the spans over which the drag check read subsonic.
fn drag_check_run(
    rows: &[Row],
    ign_s: f32,
    cd_scale: f32,
    max_open_mach: f32,
) -> (Option<(f32, bool)>, Vec<(f32, f32)>) {
    let mut config = lc25_config();
    config.max_open_mach = max_open_mach;
    // Scaling `cd` scales `Cd*A/m`, which is the only thing the check takes
    // from the airframe. `v` goes as `1/sqrt(k)`, so a k that is too HIGH
    // reads the speed LOW and calls subsonic early — the unsafe direction.
    for c in config.rocket.cd.iter_mut() {
        *c *= cd_scale;
    }
    let result = replay(rows, config);
    let birth = result
        .birth
        .map(|(t, forced)| ((t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s, forced));
    let spans = result
        .subsonic_spans
        .iter()
        .map(|(a, b)| (a - ign_s, b - ign_s))
        .collect();
    (birth, spans)
}

/// Baro-truth Mach `t_rel` seconds after ignition detection: the +-0.5 s baro
/// rate over the local speed of sound. Independent of everything the
/// estimator does — it is the log's own barometer, differentiated — which is
/// what makes it usable as the scoring reference here. LC'25 leaves the rail
/// 3.7 deg off vertical, so the vertical rate under-reads the airspeed by
/// 0.2%, far below the margins asserted below.
fn truth_mach_at(rows: &[Row], ign_s: f32, t_rel: f32) -> f32 {
    let i = rows
        .iter()
        .position(|r| (r.timestamp_us - rows[0].timestamp_us) as f32 * 1e-6 >= ign_s + t_rel)
        .expect("time is past the end of the log");
    reference_velocity(rows, i) / crate::utils::approximate_speed_of_sound(rows[i].altitude_asl)
}

/// The lockout exit must actually respect [`AirbrakesConfig::max_open_mach`],
/// and must degrade gracefully when the Cd it inverts with is wrong.
///
/// Two properties, both scored against the log's own barometer rather than
/// against a number this test computed itself:
///
/// 1. **The ceiling is honoured and load-bearing.** Sweeping it moves the
///    birth monotonically, and at every setting the airframe is genuinely at
///    or below that Mach when the filter is born. Nothing else in the suite
///    ties the check to the CONFIGURED value: every config in the crate
///    votes at 0.8, so replacing `self.config.max_open_mach` in the check
///    with a hard-coded 0.8 leaves all 48 other tests passing and only this
///    one failing. (Raising it outright to Mach 2 is caught elsewhere too —
///    `lc25_v2_replay` and `lc25_clipped_accel_replay` both notice — because
///    the birth then falls back onto the `T_min` + 1 s sustain floor at
///    ignition+9.07 s, where LC'25 is still doing Mach 0.905.)
/// 2. **A wrong drag model costs bounded margin, in a known direction.**
///    `Cd*A/m` is not a free parameter — it comes from `ROCKET_PARAMETERS`,
///    which the MPC already needs — but it is a model, and models are wrong.
#[test]
fn drag_check_timing_and_sensitivity() {
    init_logger();
    let rows = extend_pad(lc25_rows(), 12.0);
    let ign_s = t_s(&rows, find_ignition(&rows));

    // --- 1. the velocity ceiling ------------------------------------------
    let mut prev: Option<(f32, f32)> = None;
    for ceiling in [0.6f32, 0.7, 0.8, 0.9, 1.0] {
        let (birth, _) = drag_check_run(&rows, ign_s, 1.0, ceiling);
        let (born_s, forced) =
            birth.unwrap_or_else(|| panic!("ceiling {ceiling}: filter never born"));
        let mach = truth_mach_at(&rows, ign_s, born_s);
        eprintln!(
            "ceiling {ceiling:.2}: born ignition+{born_s:.2}s (forced {forced}), \
             baro-truth Mach there {mach:.3}"
        );

        // The whole point of the ceiling: the airframe really is at or below
        // it when the flaps become commandable. Measured margins run from
        // 0.068 (ceiling 0.6) to 0.156 (ceiling 1.0) of Mach, all on the safe
        // side — inverting with the SUBSONIC Cd is what buys them, since the
        // true Cd is transonically elevated and the check therefore reads
        // high exactly while it matters.
        assert!(
            mach <= ceiling,
            "ceiling {ceiling}: born at ignition+{born_s:.2}s where the baro-truth \
             Mach is {mach:.3} — ABOVE the configured ceiling, so the flaps would \
             be cleared to open in flow the airframe is not qualified for"
        );
        assert!(!forced, "ceiling {ceiling}: fell through to the T_max timeout");

        // ...and the ceiling is what decides, not the T_min floor or the
        // sustain: raising it must move the birth earlier every time.
        // Measured steps are 0.97-2.39 s apart; 0.5 s is comfortably inside
        // that and far outside the 2 ms sample spacing.
        if let Some((prev_ceiling, prev_born)) = prev {
            assert!(
                born_s + 0.5 < prev_born,
                "raising the ceiling from {prev_ceiling} to {ceiling} moved the birth \
                 only from ignition+{prev_born:.2}s to +{born_s:.2}s — the drag check \
                 is not actually voting at `max_open_mach`"
            );
        }
        prev = Some((ceiling, born_s));
    }

    // --- 2. what a wrong Cd*A/m costs -------------------------------------
    // v goes as 1/sqrt(k), so +-30% in the drag model is +-14% in the speed
    // it reads. +-30% is chosen as the honest envelope on a CFD Cd table:
    // LC'25's own flight corroborates its analytic 2.4e-4 to within about
    // 10% (0.00022-0.00026 measured across the subsonic coast).
    let mut prev: Option<(f32, f32)> = None;
    let (mut earliest, mut latest) = (f32::INFINITY, f32::NEG_INFINITY);
    for scale in [0.7f32, 0.85, 1.0, 1.15, 1.3] {
        let (birth, spans) = drag_check_run(&rows, ign_s, scale, 0.8);
        let (born_s, forced) =
            birth.unwrap_or_else(|| panic!("Cd x{scale}: filter never born"));
        let mach = truth_mach_at(&rows, ign_s, born_s);
        let (first_read_s, _) = *spans
            .first()
            .unwrap_or_else(|| panic!("Cd x{scale}: the check never read subsonic"));
        let first_read_mach = truth_mach_at(&rows, ign_s, first_read_s);
        eprintln!(
            "Cd x{scale:.2}: first read subsonic at ignition+{first_read_s:.2}s \
             (Mach {first_read_mach:.3}), born ignition+{born_s:.2}s \
             (Mach {mach:.3}, forced {forced})"
        );

        // The drag check, not the backstop, still decides across the whole
        // band — that is what "the timer is a backstop" means.
        assert!(
            !forced,
            "Cd x{scale}: a {:.0}% drag-model error pushed the exit onto the T_max \
             timeout",
            (scale - 1.0) * 100.0
        );

        // The unsafe direction, bounded. Worst measured is Mach 0.803 at
        // +30%, i.e. 0.003 over the 0.8 ceiling: a drag model wrong by a
        // third buys back only 0.05 Mach of the margin the subsonic Cd and
        // the sustain put there. 0.85 is that worst case plus room, and is
        // still below the transonic rise the port error lives in.
        assert!(
            mach < 0.85,
            "Cd x{scale}: born at baro-truth Mach {mach:.3} — a {:.0}% drag-model \
             error is opening the lockout transonically",
            (scale - 1.0) * 100.0
        );

        // The 1 s sustain is load-bearing, not decoration: the check always
        // reads subsonic first at a HIGHER true Mach than the one it
        // eventually births at (0.855 vs 0.803 in the worst case).
        assert!(
            first_read_mach > mach,
            "Cd x{scale}: the sustain bought nothing — first subsonic read at Mach \
             {first_read_mach:.3}, birth at {mach:.3}"
        );

        if let Some((prev_scale, prev_born)) = prev {
            assert!(
                born_s < prev_born,
                "Cd x{prev_scale} born at ignition+{prev_born:.2}s but x{scale} at \
                 +{born_s:.2}s — a HIGHER Cd*A/m must read the speed lower and \
                 exit earlier"
            );
        }
        prev = Some((scale, born_s));
        earliest = earliest.min(born_s);
        latest = latest.max(born_s);
    }

    // The cost of the model error, stated as a number: the whole +-30% band
    // is worth 4.85 s of control window on LC'25, against a T_min..T_max
    // window of 12 s. Bounding it at 6 s makes a drag model that has become
    // twice as sensitive fail here rather than in flight.
    eprintln!("Cd x0.7..x1.3 spans ignition+{earliest:.2}s..+{latest:.2}s of birth time");
    assert!(
        latest - earliest < 6.0,
        "a +-30% drag-model error moves the lockout exit by {:.2}s",
        latest - earliest
    );
}

/// The T_max backstop, exercised: a forced birth, which no test in this crate
/// had ever produced.
///
/// [`MachLockoutConfig::force_birth_after_ignition_us`] exists for the case
/// its own doc describes — a drag model wrong enough that the check never
/// passes, where the axial-sign burnout latch (which does not depend on Cd)
/// still fires. Six tests assert `forced == false`; until this one, nothing
/// asserted the other branch works at all, and the branch does more than set
/// a flag: it selects `FORCED_BORN_VELOCITY_STD` (30 m/s rather than 15) so
/// the barometer pulls a possibly badly drifted dead-reckoned velocity back.
///
/// What this test does NOT pin is that second effect, and the reason is worth
/// recording rather than leaving for the next person to rediscover: on LC'25
/// the dead reckoner is only 3.3 m/s out when T_max arrives, and 500 Hz of
/// 3 m baro washes the initial velocity covariance out inside half a second,
/// so 15 and 30 land on 136.79 and 136.80 m/s at birth+0.5 s — identical to
/// the precision anything here could assert. Separating them needs a log
/// where the dead reckoner is genuinely broken at T_max, which the archive
/// does not currently contain (Void Lake's clipping is on a subsonic profile
/// with no lockout at all). This test pins the branch and everything
/// downstream of it.
///
/// The wrong model here is `Cd*A/m` five times too small, which reads every
/// airspeed 2.24x too high — the same 5x error the Osiris sim uses to bound
/// [`MachLockoutConfig::earliest_subsonic_after_ignition_us`]. On LC'25 the
/// check then never reads subsonic even once, all the way to apogee, so the
/// backstop is unambiguously the only thing that can have birthed the filter.
#[test]
fn forced_birth_backstop_flies_the_rest_of_the_flight() {
    init_logger();
    let rows = extend_pad(lc25_rows(), 12.0);
    let (apogee_ref_i, apogee_ref_alt_asl) = baro_apogee(&rows);
    let ign_s = t_s(&rows, find_ignition(&rows));

    let mut config = lc25_config();
    for c in config.rocket.cd.iter_mut() {
        *c *= 0.2;
    }
    let t_max_s = match &config.mach_lockout {
        Some(l) => l.force_birth_after_ignition_us as f32 * 1e-6,
        None => panic!("this test needs a Mach lockout"),
    };
    let result = replay(&rows, config);

    let (birth_t, forced) = result.birth.expect("filter never born");
    let born_s = (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - ign_s;
    let burnout_s = result.burnout_s.expect("burnout never latched") - ign_s;
    eprintln!(
        "forced birth: born ignition+{born_s:.3}s (forced {forced}), T_max {t_max_s:.1}s, \
         burnout latch ignition+{burnout_s:.2}s, drag-check spans {:?}",
        result.subsonic_spans
    );

    assert!(forced, "the T_max backstop did not fire — birth reports forced=false");
    // If the check had ever passed, "forced" would prove nothing about the
    // backstop; with a 5x-wrong Cd it never reads subsonic at all.
    assert!(
        result.subsonic_spans.is_empty(),
        "the drag check read subsonic with a 5x-wrong Cd, so this run no longer \
         isolates the backstop"
    );
    // The backstop fires on the first sample past T_max, and not before:
    // measured ignition+20.066 s against a 20.0 s T_max, the 66 ms being the
    // detector's own lag between true ignition and the clock's origin.
    assert!(
        (t_max_s..t_max_s + 0.2).contains(&born_s),
        "forced birth at ignition+{born_s:.3}s, T_max is {t_max_s:.1}s"
    );
    // ...and it is still gated on the burnout latch, like every other birth
    // path (measured ignition+6.38 s, 13.7 s of margin).
    assert!(
        burnout_s < born_s,
        "forced birth at ignition+{born_s:.2}s preceded the burnout latch at \
         +{burnout_s:.2}s — the MPC could be handed a state under thrust"
    );

    // --- the state it starts from is sane ---------------------------------
    let at = |after: f32| -> (usize, f32, f32, f32) {
        let target = ign_s + born_s + after;
        let i = rows
            .iter()
            .position(|r| (r.timestamp_us - rows[0].timestamp_us) as f32 * 1e-6 >= target)
            .expect("past the end of the log");
        let vv = result
            .vv_track
            .iter()
            .find(|(j, _)| *j >= i)
            .map(|(_, v)| *v)
            .expect("filter died after birth");
        let alt = result
            .alt_track
            .iter()
            .find(|(j, _)| *j >= i)
            .map(|(_, a)| *a)
            .expect("no altitude after birth");
        (i, vv, alt, reference_velocity(&rows, i))
    };

    let (i_half, vv_half, alt_half, ref_half) = at(0.5);
    eprintln!(
        "forced birth +0.5s: alt {alt_half:.1} m (raw baro {:.1}), vv {vv_half:.1} m/s \
         (baro-rate ref {ref_half:.1})",
        rows[i_half].altitude_asl
    );
    // Altitude at a forced birth comes from the same 9-sample baro median as
    // any other birth, so it must land on the barometer immediately —
    // measured 0.07 m out. 20 m is the ejection-blast-free slack the
    // innovation gate itself works in.
    assert!(
        (alt_half - rows[i_half].altitude_asl).abs() < 20.0,
        "altitude {alt_half} is {:.1} m off the raw baro {:.1} half a second after a \
         forced birth",
        alt_half - rows[i_half].altitude_asl,
        rows[i_half].altitude_asl
    );
    // Velocity starts from the dead reckoner, which after 20 s of unaided
    // integration is 3.7 m/s out here. 20 m/s is the same slack
    // `lc25_clipped_accel_replay` allows a badly poisoned dead reckoner.
    assert!(
        (vv_half - ref_half).abs() < 20.0,
        "vv {vv_half} vs baro-rate reference {ref_half} half a second after a forced birth"
    );

    // Whichever initial velocity uncertainty the flag selected, the baro must
    // own the velocity channel within a couple of seconds — a forced birth is
    // the case where the dead-reckoned velocity is least trustworthy, so a
    // filter that stayed anchored to it would be the failure. Measured
    // 1.3 m/s at +2 s.
    let (_, vv_2s, _, ref_2s) = at(2.0);
    eprintln!("forced birth +2.0s: vv {vv_2s:.1} m/s vs ref {ref_2s:.1}");
    assert!(
        (vv_2s - ref_2s).abs() < 5.0,
        "two seconds after a forced birth vv is still {:.1} m/s off the reference — \
         the baro has not pulled the dead-reckoned velocity back",
        vv_2s - ref_2s
    );

    // --- and the rest of the flight still comes out right -----------------
    let from = rows
        .iter()
        .position(|r| r.timestamp_us > birth_t + 2_000_000)
        .unwrap();
    let mut to = apogee_ref_i;
    while to > 0 && t_s(&rows, apogee_ref_i) - t_s(&rows, to) < 2.0 {
        to -= 1;
    }
    let (err, n) = vv_error(&rows, &result.vv_track, from, to);
    eprintln!("forced birth: coast vv mean |err|={err:.2} m/s over {n} samples");
    assert!(n > 1000, "filter not alive through the coast");
    assert!(err < 10.0, "coast vv err {err}");

    let apogee_i = result.apogee_i.expect("apogee never latched");
    let apogee_err_s = t_s(&rows, apogee_i) - t_s(&rows, apogee_ref_i);
    let apogee_err_m = result.apogee_alt_asl.unwrap() - apogee_ref_alt_asl;
    eprintln!("forced birth: apogee {apogee_err_s:+.1}s / {apogee_err_m:+.1}m vs baro ref");
    // Same bounds as every other LC'25 replay in this file: a forced birth
    // must not cost apogee accuracy, and measured it does not (-2.3 s /
    // -15 m, against -2.3 s / -15 m for the check-born run).
    assert!(apogee_err_s.abs() < 3.0, "apogee time err {apogee_err_s}");
    assert!(apogee_err_m.abs() < 80.0, "apogee alt err {apogee_err_m}");
}

