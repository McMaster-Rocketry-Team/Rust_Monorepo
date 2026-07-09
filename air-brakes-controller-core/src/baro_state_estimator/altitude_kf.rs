use crate::baro_state_estimator::DT;
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
}

/// Altitude noise variance of the MS5607 measured on the VLF5 board
/// (bench noise floor test 2026-06-11, 4309 samples at 50 Hz, OSR=1024,
/// detrended std = 0.494 m)
pub const BARO_ALTITUDE_MEASUREMENT_VARIANCE: f32 = 0.244;

/// White-acceleration process noise driving the constant-velocity model
const PROCESS_ACCELERATION_VARIANCE: f32 = 1150.0;

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

    /// Incorporate a new barometric altitude measurement (m)
    pub fn update(&mut self, z_baro: f32) {
        let z = Vector1::new(z_baro);

        // Innovation y = z - H x̂₋
        let y = z - self.h * self.x;

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
    }

    pub fn altitude(&self) -> f32 {
        self.x[0]
    }

    pub fn vertical_velocity(&self) -> f32 {
        self.x[1]
    }
}
