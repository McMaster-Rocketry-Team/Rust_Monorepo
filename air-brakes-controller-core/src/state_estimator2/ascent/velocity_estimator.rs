use core::f32::consts::FRAC_PI_2;

use nalgebra::{Matrix2, SMatrix, SVector};

use crate::state_estimator2::DT;

const N: usize = 5; // State: [z, s, theta, omega, b_tilt]
const M: usize = 2; // Measurements: [tilt, altitude]
const EPS: f32 = 1e-5;

#[derive(Clone, Copy)]
pub struct ProcessNoiseStd {
    pub z: f32,     // model error on altitude propagation
    pub s: f32,     // speed magnitude model error (accel input uncertainty)
    pub theta: f32, // tilt model error
    pub omega: f32, // tilt-rate random walk
    pub b: f32,     // tilt-bias random walk
}

#[derive(Clone, Copy)]
pub struct MeasNoiseStd {
    pub tilt: f32, // radians
    pub alt: f32,  // meters
}

pub struct VelocityEstimator {
    x: SVector<f32, N>, // [altitude, speed magnitude, tilt, tilt rate, tilt bias]
    P: SMatrix<f32, N, N>,
    q: ProcessNoiseStd,
    r: MeasNoiseStd,

    /// When true, enforce monotonic constraints (coast-phase):
    /// speed non-increasing, tilt non-decreasing.
    pub constraints_enabled: bool,

    // Internals for constraint reference
    s_prev: f32,
    theta_prev: f32,
}

impl VelocityEstimator {
    pub fn new(
        initial_alt_asl: f32,
        initial_speed: f32,
        initial_tilt_rad: f32,
        q: ProcessNoiseStd,
        r: MeasNoiseStd,
    ) -> Self {
        let mut x = SVector::<f32, N>::zeros();
        x[0] = initial_alt_asl;
        x[1] = initial_speed.max(0.0);
        x[2] = initial_tilt_rad.clamp(0.0, FRAC_PI_2);
        x[3] = 0.0; // tilt rate
        x[4] = 0.0; // tilt bias

        // TODO change
        let mut P = SMatrix::<f32, N, N>::zeros();
        P[(0, 0)] = 10.0_f32.powi(2);
        P[(1, 1)] = 50.0_f32.powi(2);
        P[(2, 2)] = (10f32.to_radians()).powi(2);
        P[(3, 3)] = (5f32.to_radians()).powi(2);
        P[(4, 4)] = (5f32.to_radians()).powi(2);

        Self {
            x,
            P,
            q,
            r,
            constraints_enabled: false,
            s_prev: x[1],
            theta_prev: x[2],
        }
    }

    #[inline]
    pub fn v_vertical(&self) -> f32 {
        self.x[1] * self.x[2].cos()
    }
    #[inline]
    pub fn v_horizontal(&self) -> f32 {
        self.x[1] * self.x[2].sin()
    }

    #[inline]
    pub fn altitude_asl(&self) -> f32 {
        self.x[0]
    }

    #[inline]
    pub fn tilt(&self) -> f32 {
        self.x[2]
    }

    /// Predict with inputs: a_z (vertical), a_h (horizontal magnitude), both in m/s^2.
    pub fn predict(&mut self, a_z: f32, a_h: f32) {
        let z = self.x[0];
        let s = self.x[1].max(EPS);
        let theta = self.x[2];
        let omega = self.x[3];
        let b = self.x[4];

        // Raw propagation (discrete Euler)
        let z_new = z + DT * s * theta.cos();
        let s_raw = s + DT * (a_h * theta.sin() + a_z * theta.cos());
        let theta_raw = theta + DT * omega;
        let omega_new = omega; // random walk via Q
        let b_new = b; // random walk via Q

        // Always apply physical clamps
        let mut s_new = s_raw.max(0.0);
        let mut theta_new = theta_raw.clamp(0.0, core::f32::consts::FRAC_PI_2);

        // Coast-phase monotonic constraints if enabled
        let mut clamp_s = false;
        let mut clamp_theta = false;
        if self.constraints_enabled {
            if s_new > self.s_prev {
                s_new = self.s_prev;
                clamp_s = true;
            }
            if theta_new < self.theta_prev {
                theta_new = self.theta_prev;
                clamp_theta = true;
            }
        }

        // Jacobian F = ∂f/∂x at previous state
        let mut F = SMatrix::<f32, N, N>::identity();

        // z' deps
        F[(0, 1)] = DT * theta.cos();
        F[(0, 2)] = -DT * s * theta.sin();

        // s' deps; if clamped, freeze cross-term to avoid injecting nonsense
        if !clamp_s {
            F[(1, 1)] = 1.0;
            F[(1, 2)] = DT * (a_h * theta.cos() - a_z * theta.sin());
        } else {
            F[(1, 1)] = 1.0;
            F[(1, 2)] = 0.0;
        }

        // theta' deps; if clamped, drop dependence on omega
        if !clamp_theta {
            F[(2, 2)] = 1.0;
            F[(2, 3)] = DT;
        } else {
            F[(2, 2)] = 1.0;
            F[(2, 3)] = 0.0;
        }

        // Discrete process noise
        let mut Q = SMatrix::<f32, N, N>::zeros();
        Q[(0, 0)] = (self.q.z * DT).powi(2);
        Q[(1, 1)] = (self.q.s * DT).powi(2);
        Q[(2, 2)] = (self.q.theta * DT).powi(2);
        Q[(3, 3)] = (self.q.omega * DT).powi(2);
        Q[(4, 4)] = (self.q.b * DT).powi(2);

        // Commit state
        self.x[0] = z_new;
        self.x[1] = s_new.max(0.0);
        self.x[2] = theta_new;
        self.x[3] = omega_new;
        self.x[4] = b_new;

        // Covariance
        self.P = F * self.P * F.transpose() + Q;

        // Update reference values regardless of mode so enabling constraints later is seamless
        self.s_prev = self.x[1];
        self.theta_prev = self.x[2];
    }

    /// Update with available measurements. Pass None to skip a channel.
    /// Measurements: tilt_meas_rad (theta + bias), altitude_meas (z)
    pub fn update(&mut self, tilt_meas_rad: f32, altitude_meas: Option<f32>) {
        let mut H = SMatrix::<f32, M, N>::zeros();
        let mut R = Matrix2::<f32>::zeros();
        let mut y = SVector::<f32, M>::zeros();
        let mut h = SVector::<f32, M>::zeros();

        let mut rows = 1usize;

        H[(rows, 2)] = 1.0; // theta
        H[(rows, 4)] = 1.0; // bias
        y[rows] = tilt_meas_rad;
        h[rows] = self.x[2] + self.x[4];
        R[(rows, rows)] = self.r.tilt.powi(2);

        if let Some(alt) = altitude_meas {
            H[(rows, 0)] = 1.0; // z
            y[rows] = alt;
            h[rows] = self.x[0];
            R[(rows, rows)] = self.r.alt.powi(2);
            rows += 1;
        }

        let Hm = H.rows(0,rows);
        let ym = y.rows(0,rows);
        let hm = h.rows(0,rows);
        let Rm = R.view_range(0..rows, 0..rows);

        let r = ym - hm;
        let S = Hm.clone() * self.P * Hm.transpose() + Rm.clone();
        let Sinv = S.clone().try_inverse().unwrap_or_else(|| {
            let mut Sj = S.clone();
            for i in 0..rows {
                Sj[(i, i)] += 1e-6;
            }
            Sj.try_inverse().expect("Innovation matrix not invertible")
        });
        let K = self.P * Hm.transpose() * Sinv;

        let I = SMatrix::<f32, N, N>::identity();
        self.x += K.clone() * r;
        let IKH = I - K.clone() * Hm;
        self.P = IKH.clone() * self.P * IKH.transpose() + K.clone() * Rm * K.transpose();

        // Always enforce physical ranges
        self.x[1] = self.x[1].max(0.0);
        self.x[2] = self.x[2].clamp(0.0, core::f32::consts::FRAC_PI_2);
    }
}
