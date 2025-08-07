use nalgebra::{SMatrix, SVector, UnitQuaternion};

pub use state::State;
use state_transition::state_transition;

use crate::{
    RocketConstants,
    state_estimator::{
        Measurement,
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
                &[1e-4; { State::SIZE }],
            )),
            q: SMatrix::from_diagonal(
                &SVector::<f32, { State::SIZE }>::from_column_slice(&[
                    4.162997e-7,
                    3.3608598e-5,
                    3.812254e-6,
                    3.6717886e-6,
                    3.188936e-6,
                    1.8793952e-6,
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
                .map(|x| x.max(1e-10) * 10.0),
            ),
            r: SMatrix::from_diagonal(&SVector::<f32, { Measurement::SIZE }>::from_column_slice(
                &[
                    acc_variance,
                    acc_variance,
                    acc_variance,
                    gyro_variance,
                    gyro_variance,
                    gyro_variance,
                    alt_variance,
                ],
            )),
        }
    }

    pub fn predict(&mut self, airbrakes_ext: f32) {
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
        // self.orientation = self.state
        //     .reset_small_angle_correction(&self.orientation);
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
        let s = h_jacobian * self.p * h_jacobian.transpose() + self.r;
        let k = self.p * h_jacobian.transpose() * s.try_inverse().unwrap();

        self.state.0 += k * y;
        self.orientation = self.state.reset_small_angle_correction(&self.orientation);

        let i = SMatrix::<f32, { State::SIZE }, { State::SIZE }>::identity();
        self.p = (i - k * h_jacobian) * self.p;
        self.p = 0.5 * (self.p + self.p.transpose());
    }
}
