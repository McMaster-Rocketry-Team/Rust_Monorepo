use nalgebra::{SMatrix, SVector, UnitQuaternion};

pub use state::State;
use state_transition::state_transition;

use crate::{
    RocketConstants,
    state_estimator::{
        DT, Measurement,
        ascent_fusion::mekf::{
            measurement_model::{measurement_model, measurement_model_jacobian},
            state_transition::state_transition_jacobian,
        },
    },
};

mod jacobian;
mod measurement_model;
mod state;
mod state_transition;
#[cfg(test)]
mod tests;

pub struct MekfStateEstimator {
    pub orientation: UnitQuaternion<f32>,
    pub state: State,
    constants: RocketConstants,

    /// covariance P_k|k
    p: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// process-noise covariance Q
    q: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// measurement-noise covariance R
    r: SMatrix<f32, { Measurement::SIZE }, { Measurement::SIZE }>,
}

impl MekfStateEstimator {
    pub fn new(
        initial_orientation: UnitQuaternion<f32>,
        initial_state: State,
        // the variances are in imu frame
        acc_variance: f32,
        gyro_variance: f32,
        alt_variance: f32,
        constants: RocketConstants,
    ) -> Self {
        Self {
            orientation: initial_orientation,
            state: initial_state,
            constants,
            p: SMatrix::from_diagonal(&SVector::<f32, { State::SIZE }>::from_column_slice(
                &[1.0; { State::SIZE }],
            )),
            q: SMatrix::from_diagonal(
                &SVector::<f32, { State::SIZE }>::from_column_slice(&[
                    4.162997e-7,
                    3.3608598e-5,
                    3.812254e-6,
                    3.8839986e-12,
                    4.4831076e-11,
                    4.6087963e-11,
                    3.6660953e-8,
                    2.7753133e-6,
                    4.8287743e-6,
                    1.1891631e-8,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ])
                .map(|x| x.max(1e-10) * 1000.0),
            ),
            r: SMatrix::from_diagonal(&SVector::<f32, { Measurement::SIZE }>::from_column_slice(
                &[
                    // FIXME
                    1.0,
                    1.0,
                    1.0,
                    gyro_variance,
                    gyro_variance,
                    gyro_variance,
                    alt_variance,
                ],
            )),
        }
    }

    pub fn predict(&mut self, airbrakes_ext: f32) {
        // Propagate nominal orientation using current angular velocity.
        // Angular velocity in state is in world frame; convert to body frame for quaternion integration.
        let omega_world = self.state.angular_velocity();
        let omega_body = self.orientation.transform_vector(&omega_world); // TODO check
        let dq = UnitQuaternion::from_scaled_axis(omega_body * DT);
        self.orientation = self.orientation * dq;
        // Keep quaternion well-normalized.

        let f = state_transition_jacobian(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );

        // log_info!("before state: {}", self.state.0);
        self.state = state_transition(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        // log_info!("predicted state: {}", self.state.0);

        self.p = f * self.p * f.transpose() + self.q;
        self.p = 0.5 * (self.p + self.p.transpose()); // keep symmetric
    }

    pub fn update(&mut self, airbrakes_ext: f32, z_earth_frame: Measurement) {
        let h = measurement_model(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        let y = z_earth_frame.0 - h.0;
        // panic!("actual measurement: {}, predicted measurment: {}", z_earth_frame.0, self.h().0);
        let h_jacobian = measurement_model_jacobian(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        // Innovation covariance with small jitter for numerical robustness
        let mut s = h_jacobian * self.p * h_jacobian.transpose() + self.r;
        for i in 0..Measurement::SIZE {
            s[(i, i)] += 1e-6;
        }
        // Compute Kalman gain via Cholesky solve: K = P H^T S^{-1}
        let p_ht = self.p * h_jacobian.transpose(); // (n x m)
        let chol = s.cholesky().expect("S not SPD");
        let x_t = chol.solve(&p_ht.transpose()); // solves S * X = (P H^T)^T => X: (m x n)
        let k = x_t.transpose(); // (n x m)

        self.state.0 += k * y;
        // Save applied small-angle correction for covariance reset
        let delta_theta = self.state.small_angle_correction();
        self.orientation = self.state.reset_small_angle_correction(&self.orientation);

        let i = SMatrix::<f32, { State::SIZE }, { State::SIZE }>::identity();
        self.p = (i - k * h_jacobian) * self.p;
        // Apply reset Jacobian G to keep covariance consistent after injecting delta_theta
        let mut g = SMatrix::<f32, { State::SIZE }, { State::SIZE }>::identity();
        // Top-left 3x3 orientation error block: G = I - 0.5 [delta_theta]_x
        let skew = SMatrix::<f32, 3, 3>::new(
            0.0,
            -delta_theta.z,
            delta_theta.y,
            delta_theta.z,
            0.0,
            -delta_theta.x,
            -delta_theta.y,
            delta_theta.x,
            0.0,
        );
        let o3 = SMatrix::<f32, 3, 3>::identity() - 0.5 * skew;
        g.fixed_view_mut::<3, 3>(0, 0).copy_from(&o3);

        self.p = g * self.p * g.transpose();
        self.p = 0.5 * (self.p + self.p.transpose());
    }
}
