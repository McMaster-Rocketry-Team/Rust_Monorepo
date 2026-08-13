use crate::baro_state_estimator::{DT, SAMPLES_PER_S};
use micromath::F32Ext;
use nalgebra::{Matrix2, SMatrix, SVector, Vector1, Vector2};

/// Classic (linear) Kalman filter for a 1-D altitude + vertical-speed model,
/// running at [`crate::baro_state_estimator::SAMPLES_PER_S`].
///
/// State vector  x = [ altitude, vertical_speed ]ᵀ  (units: m, m s⁻¹)
/// Measurement z = barometric altitude (m)
///
///     xₖ₊₁ = F · xₖ + w,   w ~ 𝒩(0,Q)
///     zₖ   = H · xₖ + v,   v ~ 𝒩(0,R)
///
/// F = ⎡1  dt⎤ ,  H = ⎡1  0⎤
#[derive(Debug, Clone)]
pub struct BaroAltitudeKF {
    /// Current state estimate [h, v]ᵀ
    x: SVector<f32, 2>,
    /// Estimate covariance
    p: SMatrix<f32, 2, 2>,
    /// State-transition matrix
    f: SMatrix<f32, 2, 2>,
    /// Measurement matrix
    h: SMatrix<f32, 1, 2>,
    /// Process-noise covariance
    q: SMatrix<f32, 2, 2>,
    /// Measurement-noise covariance
    r: SMatrix<f32, 1, 1>,
    /// Consecutive measurements rejected by the innovation gate
    rejected_streak: u32,
}

/// Altitude noise variance of the MS5607 on the VLF5 board at pressure OSR=512.
/// Bench noise floor test 2026-06-11 (4309 samples at 50 Hz, OSR=1024) measured
/// a detrended std of 0.494 m; scaled by the datasheet RMS-noise ratio
/// OSR512/OSR1024 (0.053/0.039 mbar) -> std 0.67 m. Re-run the baro bench at
/// OSR=512 to replace this estimate with a measurement.
pub const BARO_ALTITUDE_MEASUREMENT_VARIANCE: f32 = 0.45;

/// White-acceleration process noise driving the constant-velocity model
const PROCESS_ACCELERATION_VARIANCE: f32 = 1150.0;

/// Innovation gate: reject a measurement whose innovation exceeds this. Nothing
/// the airframe can do moves barometric altitude this far in one sample; only
/// measurement faults do. On the Void Lake flight the redundant computer's
/// ejection-charge overpressure produced 200-1465 m innovations, while the
/// worst genuine dynamics mismatch (boost, at half sample rate) stayed under
/// ~45 m.
const INNOVATION_GATE_M: f32 = 75.0;

/// Force-accept after this many consecutive rejections (1 s). A transient this
/// long is not a pyro blast but a genuinely diverged filter, which must
/// re-converge instead of flying blind.
const MAX_REJECTED_SAMPLES: u32 = SAMPLES_PER_S as u32;

impl BaroAltitudeKF {
    pub fn new(initial_altitude: f32) -> Self {
        let f = Matrix2::new(1.0, DT, 0.0, 1.0);

        // Measurement matrix (altitude only)
        let h = SMatrix::<f32, 1, 2>::new(1.0, 0.0);

        // Simplified process-noise model: integrate white acceleration noise
        let q = Matrix2::new(
            0.25 * DT.powi(4),
            0.5 * DT.powi(3),
            0.5 * DT.powi(3),
            DT.powi(2),
        ) * PROCESS_ACCELERATION_VARIANCE;

        let r = SMatrix::<f32, 1, 1>::new(BARO_ALTITUDE_MEASUREMENT_VARIANCE);

        // initial uncertainty
        let p = SMatrix::<f32, 2, 2>::identity() * 0.1;

        Self {
            x: Vector2::new(initial_altitude, 0.0),
            p,
            f,
            h,
            q,
            r,
            rejected_streak: 0,
        }
    }

    /// Predict state DT seconds ahead
    pub fn predict(&mut self) {
        // x̂₋ = F x̂
        self.x = self.f * self.x;

        // P₋ = F P Fᵀ + Q
        self.p = self.f * self.p * self.f.transpose() + self.q;
        self.p = 0.5 * (self.p + self.p.transpose()); // keep symmetric
    }

    /// Incorporate a new barometric altitude measurement (m).
    /// Returns `false` if the measurement was rejected by the innovation gate
    /// (the state keeps coasting on the prediction).
    pub fn update(&mut self, z_baro: f32) -> bool {
        let z = Vector1::new(z_baro);

        // Innovation y = z - H x̂₋
        let y = z - self.h * self.x;

        if y[0].abs() > INNOVATION_GATE_M {
            if self.rejected_streak < MAX_REJECTED_SAMPLES {
                self.rejected_streak += 1;
                return false;
            }
            // An offset this persistent is not a transient: the filter itself is
            // wrong. Inflate the altitude variance so this update snaps the
            // altitude state to the measurement (velocity keeps its estimate)
            // instead of bleeding toward it at the nominal gain.
            self.p[(0, 0)] += INNOVATION_GATE_M * INNOVATION_GATE_M;
        }
        self.rejected_streak = 0;

        // Innovation covariance S = H P₋ Hᵀ + R
        let s = self.h * self.p * self.h.transpose() + self.r;

        // Kalman gain K = P₋ Hᵀ S⁻¹
        let k = self.p * self.h.transpose() * s.try_inverse().unwrap();

        // State update x̂ = x̂₋ + K y
        self.x += k * y;

        // Covariance update P = (I - K H) P₋
        let i = SMatrix::<f32, 2, 2>::identity();
        self.p = (i - k * self.h) * self.p;
        self.p = 0.5 * (self.p + self.p.transpose());
        true
    }

    pub fn altitude(&self) -> f32 {
        self.x[0]
    }

    pub fn vertical_velocity(&self) -> f32 {
        self.x[1]
    }
}
