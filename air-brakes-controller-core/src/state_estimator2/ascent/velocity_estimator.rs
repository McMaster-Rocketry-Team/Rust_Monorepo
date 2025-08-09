use nalgebra::{SMatrix, SVector, Vector2};
use crate::state_estimator2::DT; // e.g. pub const DT: f32 = 0.02;

/// State: x = [theta, omega, length]^T
/// Measurement: z = [x, y]^T = length * [sin(theta), cos(theta)]
#[derive(Debug, Clone)]
pub struct VelocityEstimator {
    x: SVector<f32, 3>,            // [θ, ω, L]
    p: SMatrix<f32, 3, 3>,         // covariance
    r: SMatrix<f32, 2, 2>,         // meas noise (2x2, isotropic)
    sigma_alpha: f32,              // angular-accel std (rad/s^2)
    sigma_ldot: f32,               // length-rate std (units/s)
    theta_prev: f32,               // for monotonic θ
    len_prev: f32,                 // for monotonic L
    inflate_on_clip: f32,          // covariance inflation after clamping
}

impl VelocityEstimator {
    /// Initialize from a Cartesian vector z0=[x,y]. omega0 is your initial angular rate guess.
    /// meas_sigma_xy is per-axis std dev in the Cartesian measurement.
    pub fn new(
        theta0: f32,
        len0: f32,
        omega0: f32,
        sigma_alpha: f32,
        sigma_ldot: f32,
        meas_sigma_xy: f32,
    ) -> Self {
        let x = SVector::<f32, 3>::new(theta0, omega0, len0);
        let p = SMatrix::<f32, 3, 3>::identity() * 1e1;
        let r = SMatrix::<f32, 2, 2>::identity() * (meas_sigma_xy * meas_sigma_xy);

        Self {
            x,
            p,
            r,
            sigma_alpha,
            sigma_ldot,
            theta_prev: theta0,
            len_prev: len0,
            inflate_on_clip: 1e-2,
        }
    }

    /// Fixed-step predict using constant DT.
    /// θ←θ + ω·DT,  ω←ω,  L←L  (plus process noise)
    fn predict(&mut self) {
        let f = SMatrix::<f32, 3, 3>::new(
            1.0, DT,  0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        );

        // Q for [θ, ω] from white angular-acceleration (σ_α)
        let sa2 = self.sigma_alpha * self.sigma_alpha;
        let q_theta = 0.25 * DT.powi(4) * sa2;
        let q_cross = 0.5  * DT.powi(3) * sa2;
        let q_omega =         DT.powi(2) * sa2;

        // Q for L from white length-rate (σ_Ldot)
        let sl2 = self.sigma_ldot * self.sigma_ldot;
        let q_len = DT.powi(2) * sl2;

        let mut q = SMatrix::<f32, 3, 3>::zeros();
        q[(0,0)] = q_theta; q[(0,1)] = q_cross;
        q[(1,0)] = q_cross; q[(1,1)] = q_omega;
        q[(2,2)] = q_len;

        self.x = f * self.x;
        self.p = f * self.p * f.transpose() + q;

        self.enforce_constraints();
    }

    /// Update with Cartesian measurement z = [x, y]^T.
    /// h(x) = [ L sin θ, L cos θ ]^T
    pub fn update(&mut self, z: Vector2<f32>) {
        self.predict();

        let (theta, _omega, len) = (self.x[0], self.x[1], self.x[2]);
        let (st, ct) = (theta.sin(), theta.cos());

        // Predicted measurement
        let zhat = Vector2::new(len * st, len * ct);

        // Jacobian H = ∂h/∂x (2x3)
        // [ [ L cosθ, 0, sinθ ],
        //   [ -L sinθ, 0, cosθ ] ]
        let h = SMatrix::<f32, 2, 3>::new(
             len * ct, 0.0, st,
            -len * st, 0.0, ct,
        );

        // Innovation
        let y = z - zhat;

        // Innovation covariance and gain
        let s = h * self.p * h.transpose() + self.r;
        let k = self.p * h.transpose() * s.try_inverse().expect("S must be PD");

        // Joseph update for stability
        self.x += k * y;
        let i3 = SMatrix::<f32, 3, 3>::identity();
        let kh = &k * &h;
        self.p = (i3 - &kh) * self.p * (i3 - &kh).transpose() + &k * self.r * k.transpose();

        self.enforce_constraints();
    }

    #[inline]
    pub fn state(&self) -> (Vector2<f32>, f32) {
        let theta = self.x[0];
        let omega = self.x[1];
        let len   = self.x[2];
    
        // θ=0 → (0, +1); clockwise-positive
        let xy = Vector2::new(len * theta.sin(), len * theta.cos());
        (xy, omega)
    }

    fn enforce_constraints(&mut self) {
        let mut clamped = false;

        // θ ∈ [0, π/2], nondecreasing (clockwise-positive by convention)
        let theta_max = core::f32::consts::FRAC_PI_2;
        if self.x[0] < self.theta_prev { self.x[0] = self.theta_prev; clamped = true; }
        if self.x[0] < 0.0            { self.x[0] = 0.0;           clamped = true; }
        if self.x[0] > theta_max      { self.x[0] = theta_max;      clamped = true; }

        // L ≥ 0, nonincreasing
        if self.x[2] > self.len_prev  { self.x[2] = self.len_prev;  clamped = true; }
        if self.x[2] < 0.0            { self.x[2] = 0.0;            clamped = true; }

        if clamped {
            // modest inflation to avoid overconfidence after projection
            let d = SVector::<f32, 3>::new(self.inflate_on_clip, 0.0, self.inflate_on_clip);
            self.p += SMatrix::<f32, 3, 3>::from_diagonal(&d);
        }

        self.theta_prev = self.x[0];
        self.len_prev   = self.x[2];
    }
}
