use crate::baro_gate::BaroGateOutcome;
use crate::baro_state_estimator::{DT, SAMPLES_PER_S};

/// Classic (linear) Kalman filter for a 1-D altitude + vertical-speed model,
/// running at [`crate::baro_state_estimator::SAMPLES_PER_S`].
///
/// State vector  x = [ altitude, vertical_speed ]ᵀ  (units: m, m s⁻¹)
/// Measurement z = barometric altitude (m)
///
/// ```text
/// xₖ₊₁ = F · xₖ + w,   w ~ 𝒩(0,Q)
/// zₖ   = H · xₖ + v,   v ~ 𝒩(0,R)
/// ```
///
/// F = ⎡1  dt⎤ ,  H = ⎡1  0⎤
///     ⎣0   1⎦
///
/// F, H, Q and R never change, so the filter is written out in closed scalar
/// form rather than as generic 2×2 matrix products: folding away F's and H's
/// zeros and ones by hand is the whole of what those products were computing,
/// 416 times a second. The arithmetic is bit-identical to the matrix form this
/// replaces (checked over 1.46 M filter states of replayed and synthetic
/// data), and `predict` and `update` below carry the operation orderings that
/// identity depends on.
#[derive(Debug, Clone)]
pub struct BaroAltitudeKF {
    /// Current altitude estimate, x[0] (m ASL)
    altitude: f32,
    /// Current vertical-speed estimate, x[1] (m s⁻¹)
    velocity: f32,
    /// Estimate covariance P, upper triangle only: P is symmetric by
    /// construction — `update` restores symmetry every sample and `predict`
    /// preserves it — so p10 is always bit-for-bit p01 and is not stored.
    p00: f32,
    p01: f32,
    p11: f32,
    /// Consecutive measurements rejected by the innovation gate
    rejected_streak: u32,
}

/// Altitude noise variance of the MS5607 on the VLF5 board at pressure OSR=512.
/// Bench noise floor test 2026-06-11 (4309 samples at 50 Hz, OSR=1024) measured
/// a detrended std of 0.494 m; scaled by the datasheet RMS-noise ratio
/// OSR512/OSR1024 (0.053/0.039 mbar) -> std 0.67 m. Re-run the baro bench at
/// OSR=512 to replace this estimate with a measurement.
pub const BARO_ALTITUDE_MEASUREMENT_VARIANCE: f32 = 0.45;

/// White-acceleration process noise driving the constant-velocity model.
/// Sets the filter bandwidth: 0.115 gives an altitude time constant of ~1.0 s
/// (steady-state gains ~0.0024 alt / ~0.0012 vel per sample). This filter is
/// the *deployment* estimator and is deliberately slow — its outputs are
/// trusted by the flight state machine, and robustness comes from bandwidth
/// rather than transient detectors. It lags badly during dynamics (boost lag
/// ~100-330 m for 5-16 g motors, coast velocity reads ~12 m/s high) — that is
/// accepted; nothing latency-sensitive consumes it. The airbrakes get their
/// own fast estimator.
const PROCESS_ACCELERATION_VARIANCE: f32 = 0.115;

const DT2: f32 = DT * DT;

/// Process-noise covariance Q, upper triangle: white acceleration integrated
/// over one `DT` step, ⎡dt⁴/4  dt³/2⎤ · σ²ₐ.
///                    ⎣dt³/2   dt² ⎦
///
/// Written as plain multiplications rather than `DT.powi(n)`: the method form
/// resolves to a different implementation under `no_std` than under std (see
/// `utils::approximate_air_density`), and these are exact either way as
/// products. The left-to-right grouping is the one the matrix form used and
/// is load-bearing for bit-identity — `(0.25 · dt²) · dt²`, then `· σ²ₐ`.
const Q00: f32 = 0.25 * DT2 * DT2 * PROCESS_ACCELERATION_VARIANCE;
const Q01: f32 = 0.5 * DT2 * DT * PROCESS_ACCELERATION_VARIANCE;
const Q11: f32 = DT2 * PROCESS_ACCELERATION_VARIANCE;

/// Innovation gate: reject a measurement whose innovation exceeds this. Pure
/// input validation for the raw bus (a bad SPI read decoding to pressure~0 is
/// a ~30 km innovation) and for large blast transients (Void Lake ejection
/// overpressure: 200-1465 m). Sized above the slow filter's worst *genuine*
/// tracking lag — ~330 m during a 16 g subsonic boost with no Mach lockout —
/// so it never rejects real flight data. Transients that slip under it barely
/// move the slow filter (a 25-sample 500 m offset shifts altitude ~30 m and
/// velocity ~15 m/s, and no deployment decision reads short-term velocity).
const INNOVATION_GATE_M: f32 = 500.0;

/// Force-accept after this many consecutive rejections (1 s). A transient this
/// long is not a pyro blast but a genuinely diverged filter, which must
/// re-converge instead of flying blind.
const MAX_REJECTED_SAMPLES: u32 = SAMPLES_PER_S as u32;

/// Velocity variance used by [`BaroAltitudeKF::born_in_flight`]: after a Mach lockout
/// the velocity is unknown; (300 m/s)^2 covers any post-lockout speed and lets
/// the filter pull the true velocity out of the altitude stream within a few
/// hundred ms.
const RESEED_VELOCITY_VARIANCE: f32 = 300.0 * 300.0;

impl BaroAltitudeKF {
    pub fn new(initial_altitude_asl: f32) -> Self {
        Self {
            altitude: initial_altitude_asl,
            velocity: 0.0,
            // initial uncertainty
            p00: 0.1,
            p01: 0.0,
            p11: 0.1,
            rejected_streak: 0,
        }
    }

    /// Predict state DT seconds ahead
    pub fn predict(&mut self) {
        // x̂₋ = F x̂
        self.altitude += self.velocity * DT;

        // P₋ = F P Fᵀ + Q, with F's zeros and ones folded out:
        //   FP   = ⎡p00 + dt·p01   p01 + dt·p11⎤
        //          ⎣     p01            p11    ⎦
        //   FPFᵀ = ⎡(FP)₀₀ + dt·(FP)₀₁   (FP)₀₁⎤
        //          ⎣     (FP)₀₁            p11 ⎦
        //
        // Both off-diagonals of FPFᵀ come out of the same expression over the
        // same operands, and Q's are one constant, so a predict cannot break
        // P's symmetry. The explicit `0.5·(P + Pᵀ)` that used to close this
        // method was therefore a bit-exact no-op on every sample — halving
        // `p + p` is exact, so it did not touch the diagonal either — and is
        // gone. The *update* is where symmetry genuinely has to be restored;
        // see there.
        let fp00 = self.p00 + self.p01 * DT;
        let fp01 = self.p01 + self.p11 * DT;
        self.p00 = fp00 + DT * fp01 + Q00;
        self.p01 = fp01 + Q01;
        self.p11 += Q11;
    }

    /// Incorporate a new barometric altitude measurement (m). The returned
    /// [`BaroGateOutcome`] is the only report of what the gate did — a resync
    /// happens on one sample and is not recoverable by polling afterwards.
    pub fn update(&mut self, z_baro: f32) -> BaroGateOutcome {
        // Innovation y = z - H x̂₋
        let y = z_baro - self.altitude;

        let mut outcome = BaroGateOutcome::Accepted;
        if y.abs() > INNOVATION_GATE_M {
            if self.rejected_streak < MAX_REJECTED_SAMPLES {
                self.rejected_streak += 1;
                return BaroGateOutcome::Rejected;
            }
            // An offset this persistent is not a transient: the filter itself is
            // wrong. Inflate the altitude variance so this update snaps the
            // altitude state to the measurement (velocity keeps its estimate)
            // instead of bleeding toward it at the nominal gain.
            self.p00 += INNOVATION_GATE_M * INNOVATION_GATE_M;
            outcome = BaroGateOutcome::Resynced;
        }
        self.rejected_streak = 0;

        // Innovation covariance S = H P₋ Hᵀ + R, a scalar: p00 + R. p00 is a
        // variance and R is 0.45, so S >= 0.45 and the reciprocal below can
        // never blow up. (The matrix form's `try_inverse().unwrap()` here was
        // unreachable for exactly that reason: nalgebra's 1×1 `try_inverse`
        // carries no epsilon and returns `None` only at exactly ±0.0.)
        let s = self.p00 + BARO_ALTITUDE_MEASUREMENT_VARIANCE;

        // Kalman gain K = P₋ Hᵀ S⁻¹ = ⎡p00⎤ · (1/S).
        //                             ⎣p01⎦
        // Reciprocal-then-multiply, never `/ s`. `try_inverse` built 1/S and
        // multiplied by it, and the two are not the same f32: replacing this
        // with a division moves 40.4% of the altitudes and 99.6% of the
        // velocities a Void Lake replay produces (by up to 1.4 mm and
        // 0.4 mm s⁻¹).
        let s_inv = 1.0 / s;
        let k0 = self.p00 * s_inv;
        let k1 = self.p01 * s_inv;

        // State update x̂ = x̂₋ + K y
        self.altitude += k0 * y;
        self.velocity += k1 * y;

        // Covariance update P = (I - K H) P₋, which for H = [1 0] is
        //   ⎡ (1-k0)·p00   (1-k0)·p01 ⎤
        //   ⎣p01 - k1·p00  p11 - k1·p01⎦
        //
        // The two off-diagonals are one number reached by two different
        // expressions, so they disagree in the last bits: 178380 of the
        // 186872 accepted samples in a Void Lake replay (95.5%) land here
        // asymmetric. Averaging them is what hands the next `predict` the
        // symmetric P it assumes, and it is the one symmetrization this filter
        // cannot drop — dropping it moves 43.1% of that replay's altitudes and
        // 98.9% of its velocities (by up to 1.6 mm and 0.4 mm s⁻¹). The
        // diagonal half of the old `0.5·(P + Pᵀ)` was exact, so it is simply
        // not computed.
        let one_minus_k0 = 1.0 - k0;
        let p01_upper = self.p01 * one_minus_k0;
        let p01_lower = self.p01 - k1 * self.p00;
        self.p11 -= k1 * self.p01;
        self.p00 *= one_minus_k0;
        self.p01 = 0.5 * (p01_upper + p01_lower);
        outcome
    }

    /// Build a filter for a rocket that is already flying — the Mach
    /// lockout has just ended and this is the first honest baro reading
    /// since ignition.
    ///
    /// Same altitude seed as [`Self::new`], but the velocity prior is wide
    /// open rather than "at rest": the airframe is doing a few hundred m/s
    /// and a confident zero would take a second of altitude data to argue
    /// down through the innovation gate.
    ///
    /// A constructor and not a reset, because nothing survives the lockout.
    /// The estimator drops its filter outright at ignition rather than
    /// freezing one, so there is no stale state here to overwrite — see
    /// [`RocketStateEstimator::update`](super::RocketStateEstimator::update).
    pub fn born_in_flight(altitude_asl: f32) -> Self {
        let mut kf = Self::new(altitude_asl);
        kf.p00 = BARO_ALTITUDE_MEASUREMENT_VARIANCE;
        kf.p01 = 0.0;
        kf.p11 = RESEED_VELOCITY_VARIANCE;
        kf
    }

    pub fn altitude_asl(&self) -> f32 {
        self.altitude
    }

    pub fn vertical_velocity(&self) -> f32 {
        self.velocity
    }
}
