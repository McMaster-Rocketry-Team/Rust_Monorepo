//! Scratch study: would a FASTER BARO-ONLY Kalman filter replace the
//! accel-aided vertical filter the airbrakes estimator flies?
//!
//! Replays both archived flights (Void Lake, LC'25) and scores, over the
//! window the airbrakes may actually act in (birth -> apogee):
//!
//!   * the flown estimator (IMU-aided predict, tau ~1.73 s on the baro), and
//!   * baro-only constant-velocity filters at a sweep of time constants,
//!     born at the same instant with the same altitude seed.
//!
//! Truth proxy is a NON-CAUSAL quadratic fit of the same baro over +-0.4 s:
//! zero lag, unbiased in altitude and velocity under constant acceleration.
//! It cannot see baro's own systematic (static-port) error, so this is a
//! fair test of LAG and NOISE only -- which is exactly the bandwidth
//! question.
//!
//! Run: cargo run --release --example kf_bandwidth_study

use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::{Vector2, Vector3};

use air_brakes_controller_core::{
    AirBrakesMPC, ImuSample, RocketParameters,
    airbrakes_estimator::{AirbrakesConfig, AirbrakesEstimator, MachLockoutConfig},
};

// ---------------------------------------------------------------- log rows

struct Row {
    timestamp_us: u64,
    imu: ImuSample,
    altitude_asl: f32,
}

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
        .map(|r| Row {
            timestamp_us: r.timestamp_us,
            imu: ImuSample {
                acc: Vector3::new(r.acc_x, r.acc_y, r.acc_z),
                gyro: Vector3::new(
                    r.gyro_x.to_radians(),
                    r.gyro_y.to_radians(),
                    r.gyro_z.to_radians(),
                ),
            },
            altitude_asl: calculate_isa_altitude(Pascals(r.pressure as f64)).0 as f32,
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
        .map(|r| Row {
            timestamp_us: r.time_us,
            imu: ImuSample {
                acc: Vector3::new(r.imu_acc_x, -r.imu_acc_y, -r.imu_acc_z),
                gyro: Vector3::new(
                    r.gyro_x.to_radians(),
                    r.gyro_y.to_radians(),
                    r.gyro_z.to_radians(),
                ),
            },
            altitude_asl: r.altitude,
        })
        .collect()
}

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

/// Loop each log's own pad segment in front so pad calibration can finish
/// (same trick the replay tests use).
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

fn t_s(rows: &[Row], i: usize) -> f32 {
    (rows[i].timestamp_us - rows[0].timestamp_us) as f32 * 1e-6
}

// ------------------------------------------------------- zero-lag reference

/// Non-causal quadratic least-squares fit of baro altitude over +-`half_s`,
/// evaluated at the centre: (altitude, vertical velocity). Zero lag, and
/// unbiased in both under constant acceleration.
fn reference(rows: &[Row], i: usize, half_s: f32) -> Option<(f32, f32)> {
    let t0 = rows[i].timestamp_us;
    let half_us = (half_s * 1e6) as u64;
    let mut lo = i;
    while lo > 0 && t0.saturating_sub(rows[lo - 1].timestamp_us) <= half_us {
        lo -= 1;
    }
    let mut hi = i;
    while hi + 1 < rows.len() && rows[hi + 1].timestamp_us.saturating_sub(t0) <= half_us {
        hi += 1;
    }
    if hi - lo < 20 {
        return None;
    }
    // normal equations for y = a + b t + c t^2, t relative to centre
    let (mut s0, mut s1, mut s2, mut s3, mut s4) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let (mut y0, mut y1, mut y2) = (0.0f64, 0.0f64, 0.0f64);
    for j in lo..=hi {
        let t = (rows[j].timestamp_us as f64 - t0 as f64) * 1e-6;
        let y = rows[j].altitude_asl as f64;
        let (t2, t3, t4) = (t * t, t * t * t, t * t * t * t);
        s0 += 1.0;
        s1 += t;
        s2 += t2;
        s3 += t3;
        s4 += t4;
        y0 += y;
        y1 += y * t;
        y2 += y * t2;
    }
    // solve 3x3 by Cramer
    let m = [[s0, s1, s2], [s1, s2, s3], [s2, s3, s4]];
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-9 {
        return None;
    }
    let rhs = [y0, y1, y2];
    let col = |c: usize| {
        let mut a = m;
        for r in 0..3 {
            a[r][c] = rhs[r];
        }
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    };
    Some(((col(0) / det) as f32, (col(1) / det) as f32))
}

/// (index, altitude) of the highest 1 s-smoothed baro altitude.
fn baro_apogee(rows: &[Row]) -> (usize, f32) {
    let mut best = (0usize, f32::MIN);
    for i in 50..rows.len().saturating_sub(50) {
        let avg: f32 =
            rows[(i - 50)..(i + 50)].iter().map(|r| r.altitude_asl).sum::<f32>() / 100.0;
        if avg > best.1 {
            best = (i, avg);
        }
    }
    best
}

// ------------------------------------------------------- baro-only filter

/// Plain 2-state constant-velocity KF on baro alone -- no accelerometer
/// input at all. Same gate/measured-dt discipline as the flown filter.
struct BaroOnlyKF {
    alt: f32,
    vel: f32,
    p: [f32; 4], // row-major 2x2
    q_accel_std: f32,
    r_alt_std: f32,
    rejected_s: f32,
}

const GATE_M: f32 = 100.0;
const MAX_REJECTED_S: f32 = 2.0;

impl BaroOnlyKF {
    /// `tau` is the closed-loop altitude time constant: for this filter
    /// tau = sqrt(r_std / (2 q_std)), independent of sample rate.
    fn born(alt: f32, vel: f32, vel_std: f32, tau_s: f32, r_alt_std: f32) -> Self {
        Self {
            alt,
            vel,
            p: [r_alt_std * r_alt_std, 0.0, 0.0, vel_std * vel_std],
            q_accel_std: r_alt_std / (2.0 * tau_s * tau_s),
            r_alt_std,
            rejected_s: 0.0,
        }
    }

    fn predict(&mut self, dt: f32) {
        self.alt += self.vel * dt;
        let (p00, p01, p10, p11) = (self.p[0], self.p[1], self.p[2], self.p[3]);
        let q = self.q_accel_std * self.q_accel_std;
        self.p[0] = p00 + dt * (p01 + p10) + dt * dt * p11 + q * dt * dt * dt * dt / 4.0;
        self.p[1] = p01 + dt * p11 + q * dt * dt * dt / 2.0;
        self.p[2] = p10 + dt * p11 + q * dt * dt * dt / 2.0;
        self.p[3] = p11 + q * dt * dt;
    }

    fn update(&mut self, z: f32, dt: f32) {
        let innovation = z - self.alt;
        if innovation.abs() > GATE_M {
            self.rejected_s += dt;
            if self.rejected_s >= MAX_REJECTED_S {
                self.alt = z;
                self.p[0] = self.r_alt_std * self.r_alt_std;
                self.p[1] = 0.0;
                self.p[2] = 0.0;
                self.p[3] += 20.0 * 20.0;
                self.rejected_s = 0.0;
            }
            return;
        }
        self.rejected_s = 0.0;
        let r = self.r_alt_std * self.r_alt_std;
        let s = self.p[0] + r;
        let k0 = self.p[0] / s;
        let k1 = self.p[2] / s;
        self.alt += k0 * innovation;
        self.vel += k1 * innovation;
        let (p00, p01, p10, p11) = (self.p[0], self.p[1], self.p[2], self.p[3]);
        let a = 1.0 - k0;
        self.p[0] = a * a * p00 + k0 * k0 * r;
        self.p[1] = a * (p01 - k1 * p00) + k0 * k1 * r;
        self.p[2] = a * (p10 - k1 * p00) + k0 * k1 * r;
        self.p[3] = p11 - k1 * (p01 + p10) + k1 * k1 * p00 + k1 * k1 * r;
    }
}

/// Causal seed a baro-only design would have at birth: least-squares slope
/// of the last `span_s` of baro, and the fit's value at the newest sample.
fn causal_seed(rows: &[Row], i: usize, span_s: f32) -> (f32, f32) {
    let t0 = rows[i].timestamp_us;
    let span_us = (span_s * 1e6) as u64;
    let mut lo = i;
    while lo > 0 && t0.saturating_sub(rows[lo - 1].timestamp_us) <= span_us {
        lo -= 1;
    }
    let n = (i - lo + 1) as f64;
    let (mut st, mut sy, mut stt, mut sty) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for j in lo..=i {
        let t = (rows[j].timestamp_us as f64 - t0 as f64) * 1e-6;
        let y = rows[j].altitude_asl as f64;
        st += t;
        sy += y;
        stt += t * t;
        sty += t * y;
    }
    let den = n * stt - st * st;
    if den.abs() < 1e-9 {
        return (rows[i].altitude_asl, 0.0);
    }
    let slope = (n * sty - st * sy) / den;
    let intercept = (sy - slope * st) / n;
    (intercept as f32, slope as f32)
}

// ------------------------------------------------------------------ scoring

#[derive(Default)]
struct Score {
    n: usize,
    vv_abs_sum: f32,
    vv_max: f32,
    alt_abs_sum: f32,
    alt_max: f32,
    /// RMS of sample-to-sample velocity change, scaled to m/s per 10 ms --
    /// the jitter the MPC sees.
    jitter_sq_sum: f32,
    jitter_n: usize,
    ap_abs_sum: f32,
    ap_max: f32,
    ext_abs_sum: f32,
    ext_max: f32,
    /// worst error on a sample right after a >50 ms sensor gap
    stall_vv_max: f32,
}

impl Score {
    fn push(
        &mut self,
        vv_err: f32,
        alt_err: f32,
        ap_err: f32,
        ext_err: f32,
        after_stall: bool,
    ) {
        self.n += 1;
        self.vv_abs_sum += vv_err.abs();
        self.vv_max = self.vv_max.max(vv_err.abs());
        self.alt_abs_sum += alt_err.abs();
        self.alt_max = self.alt_max.max(alt_err.abs());
        self.ap_abs_sum += ap_err.abs();
        self.ap_max = self.ap_max.max(ap_err.abs());
        self.ext_abs_sum += ext_err.abs();
        self.ext_max = self.ext_max.max(ext_err.abs());
        if after_stall {
            self.stall_vv_max = self.stall_vv_max.max(vv_err.abs());
        }
    }
    fn push_jitter(&mut self, dv: f32, dt: f32) {
        if dt > 1e-6 {
            let per_10ms = dv / dt * 0.01;
            self.jitter_sq_sum += per_10ms * per_10ms;
            self.jitter_n += 1;
        }
    }
    fn row(&self, label: &str) {
        let n = self.n.max(1) as f32;
        println!(
            "  {:<26} {:>7.2} {:>7.2} {:>7.2} {:>7.2} {:>8.3} {:>8.1} {:>8.1} {:>7.1} {:>7.1} {:>8.2}",
            label,
            self.vv_abs_sum / n,
            self.vv_max,
            self.alt_abs_sum / n,
            self.alt_max,
            (self.jitter_sq_sum / self.jitter_n.max(1) as f32).sqrt(),
            self.ap_abs_sum / n,
            self.ap_max,
            self.ext_abs_sum / n * 100.0,
            self.ext_max * 100.0,
            self.stall_vv_max,
        );
    }
}

fn lc25_rocket() -> RocketParameters {
    RocketParameters {
        burnout_mass: 17.607,
        cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
        reference_area: 0.008982476,
    }
}

fn run_flight(name: &str, rows: Vec<Row>, config: AirbrakesConfig) {
    let rocket = lc25_rocket();
    let (apogee_i, apogee_alt) = baro_apogee(&rows);
    let ign_i = find_ignition(&rows);

    // ---- pass 1: the flown estimator, and where it is born
    let mut est = AirbrakesEstimator::new(config.clone());
    let mut flown: Vec<(usize, f32, Vector2<f32>)> = Vec::new();
    let mut birth: Option<(u64, bool, usize)> = None;
    for (i, r) in rows.iter().enumerate() {
        est.update(r.timestamp_us, &r.imu, r.altitude_asl);
        if birth.is_none()
            && let Some((t, forced)) = est.birth()
        {
            birth = Some((t, forced, i));
        }
        if let (Some(a), Some(v)) = (est.altitude_asl(), est.velocity()) {
            flown.push((i, a, v));
        }
    }
    let (birth_t, forced, birth_i) = birth.expect("filter never born");

    // MPC target: 150 m under the stowed prediction at the scoring start,
    // so the command sits in the live (unclamped) region.
    let score_from = rows
        .iter()
        .position(|r| r.timestamp_us > birth_t + 500_000)
        .unwrap();
    let score_to = {
        let mut to = apogee_i;
        while to > 0 && t_s(&rows, apogee_i) - t_s(&rows, to) < 0.5 {
            to -= 1;
        }
        to
    };
    let (ref_alt0, ref_vv0) = reference(&rows, score_from, 0.4).unwrap();
    let stowed = AirBrakesMPC::new(rocket.clone(), 0.0)
        .update(ref_alt0, Vector2::new(0.0, ref_vv0))
        .predicted_apogee_asl;
    let target = {
        // predicted_apogee at target 0 is the fully-braked one; get the
        // stowed prediction by targeting something unreachably high
        let high = AirBrakesMPC::new(rocket.clone(), 1e9)
            .update(ref_alt0, Vector2::new(0.0, ref_vv0))
            .predicted_apogee_asl;
        let _ = stowed;
        high - 150.0
    };
    let mpc = AirBrakesMPC::new(rocket.clone(), target);

    println!(
        "\n=== {name} ===\n  ignition t={:.1}s | filter born ignition+{:.2}s (forced: {forced}) \
         | baro apogee t={:.1}s alt={:.0}m | scoring {:.1}s..{:.1}s ({} samples) | MPC target {:.0}m",
        t_s(&rows, ign_i),
        (birth_t - rows[0].timestamp_us) as f32 * 1e-6 - t_s(&rows, ign_i),
        t_s(&rows, apogee_i),
        apogee_alt,
        t_s(&rows, score_from),
        t_s(&rows, score_to),
        score_to - score_from,
        target,
    );

    // reference + oracle command, per sample in the window
    let mut refs: Vec<Option<(f32, f32, f32, f32)>> = vec![None; rows.len()]; // alt, vv, ap, ext
    for i in score_from..score_to {
        if let Some((a, v)) = reference(&rows, i, 0.4) {
            let sol = mpc.update(a, Vector2::new(0.0, v));
            refs[i] = Some((a, v, sol.predicted_apogee_asl, sol.extension_percentage));
        }
    }

    let after_stall = |i: usize| -> bool {
        i > 0 && rows[i].timestamp_us - rows[i - 1].timestamp_us > 50_000
    };

    println!(
        "  {:<26} {:>7} {:>7} {:>7} {:>7} {:>8} {:>8} {:>8} {:>7} {:>7} {:>8}",
        "estimator", "|dvv|", "max", "|dalt|", "max", "jitter", "|dapo|", "max", "|dext|", "max",
        "stall"
    );
    println!(
        "  {:<26} {:>7} {:>7} {:>7} {:>7} {:>8} {:>8} {:>8} {:>7} {:>7} {:>8}",
        "", "m/s", "m/s", "m", "m", "m/s/10ms", "m", "m", "%pt", "%pt", "m/s"
    );

    // ---- flown estimator score
    let mut s = Score::default();
    let mut prev: Option<(u64, f32)> = None;
    for (i, alt, v) in &flown {
        let i = *i;
        if i < score_from || i >= score_to {
            continue;
        }
        if let Some((ra, rv, rap, rext)) = refs[i] {
            let sol = mpc.update(*alt, *v);
            s.push(
                v.y - rv,
                alt - ra,
                sol.predicted_apogee_asl - rap,
                sol.extension_percentage - rext,
                after_stall(i),
            );
            let _ = rext;
        }
        if let Some((pt, pv)) = prev {
            s.push_jitter(v.y - pv, (rows[i].timestamp_us - pt) as f32 * 1e-6);
        }
        prev = Some((rows[i].timestamp_us, v.y));
    }
    s.row("flown (IMU-aided, t=1.73s)");

    // ---- baro-only sweep, born at the same instant
    for tau in [0.10f32, 0.20, 0.35, 0.50, 0.75, 1.00, 1.73] {
        let (seed_alt, seed_vv) = causal_seed(&rows, birth_i, 0.5);
        let mut kf = BaroOnlyKF::born(seed_alt, seed_vv, 30.0, tau, 3.0);
        let mut s = Score::default();
        let mut prev: Option<(u64, f32)> = None;
        for i in (birth_i + 1)..score_to {
            let dt = ((rows[i].timestamp_us - rows[i - 1].timestamp_us) as f32 * 1e-6).min(0.25);
            kf.predict(dt);
            kf.update(rows[i].altitude_asl, dt);
            if i < score_from {
                prev = Some((rows[i].timestamp_us, kf.vel));
                continue;
            }
            if let Some((ra, rv, rap, rext)) = refs[i] {
                // baro-only has no attitude: horizontal velocity is 0
                let sol = mpc.update(kf.alt, Vector2::new(0.0, kf.vel));
                s.push(
                    kf.vel - rv,
                    kf.alt - ra,
                    sol.predicted_apogee_asl - rap,
                    sol.extension_percentage - rext,
                    after_stall(i),
                );
            }
            if let Some((pt, pv)) = prev {
                s.push_jitter(kf.vel - pv, (rows[i].timestamp_us - pt) as f32 * 1e-6);
            }
            prev = Some((rows[i].timestamp_us, kf.vel));
        }
        s.row(&format!("baro-only tau={tau:.2}s"));
    }

    // ---- what the IMU contributes beyond the KF predict: horizontal
    // velocity from tilt (a baro-only design cannot have it at all)
    let mut max_vx = 0.0f32;
    let mut sum_vx = 0.0f32;
    let mut n_vx = 0usize;
    for (i, _, v) in &flown {
        if *i >= score_from && *i < score_to {
            max_vx = max_vx.max(v.x);
            sum_vx += v.x;
            n_vx += 1;
        }
    }
    println!(
        "  tilt-derived horizontal velocity over the window: mean {:.1} m/s, max {:.1} m/s",
        sum_vx / n_vx.max(1) as f32,
        max_vx
    );

    // ---- what a baro-only design cannot gate: run it from ignition
    let (seed_alt, seed_vv) = causal_seed(&rows, ign_i, 0.5);
    let mut kf = BaroOnlyKF::born(seed_alt, seed_vv, 30.0, 0.35, 3.0);
    let mut worst_alt = 0.0f32;
    let mut worst_t = 0.0f32;
    for i in (ign_i + 1)..score_to {
        let dt = ((rows[i].timestamp_us - rows[i - 1].timestamp_us) as f32 * 1e-6).min(0.25);
        kf.predict(dt);
        kf.update(rows[i].altitude_asl, dt);
        if let Some((ra, _)) = reference(&rows, i, 0.4) {
            let e = (kf.alt - ra).abs();
            if e > worst_alt {
                worst_alt = e;
                worst_t = t_s(&rows, i) - t_s(&rows, ign_i);
            }
        }
    }
    // and where it stands at the moment the real filter is born
    println!(
        "  baro-only tau=0.35s run from IGNITION (no lockout possible): worst altitude error \
         {worst_alt:.0} m at ignition+{worst_t:.1}s"
    );
}

fn main() {
    let base = AirbrakesConfig {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        mach_lockout: None,
        max_open_mach: 0.8,
        rocket: lc25_rocket(),
    };

    run_flight(
        "Void Lake (subsonic, 104 ms sensor stalls)",
        extend_pad(void_lake_rows(), 8.0),
        base.clone(),
    );

    run_flight(
        "LC'25 (Mach 2, 500 Hz recorder)",
        extend_pad(lc25_rows(), 12.0),
        AirbrakesConfig {
            mach_lockout: Some(MachLockoutConfig {
                earliest_subsonic_after_ignition_us: 8_000_000,
                force_birth_after_ignition_us: 20_000_000,
            }),
            ..base
        },
    );
}
