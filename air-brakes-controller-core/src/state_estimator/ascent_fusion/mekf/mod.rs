use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

pub use state::State;
use state_propagation::{build_measurement_matrix, central_difference_jacobian, state_transition};

use crate::{
    RocketConstants,
    state_estimator::{DT, Measurement},
};

mod state;
mod state_propagation;
#[cfg(test)]
mod tests;

pub struct MekfStateEstimator {
    orientation: UnitQuaternion<f32>,
    state: State,
    constants: RocketConstants,

    /// covariance P_k|k
    p: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// process-noise covariance Q
    q: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// measurement-noise covariance R
    r: SMatrix<f32, { Measurement::SIZE }, { Measurement::SIZE }>,

    /// measurement matrix
    h: SMatrix<f32, { Measurement::SIZE }, { State::SIZE }>,
}

impl MekfStateEstimator {
    pub fn new(
        initial_orientation: UnitQuaternion<f32>,
        initial_state: State,
        // the variances are in imu frame
        acc_variance: Vector3<f32>,
        gyro_variance: Vector3<f32>,
        alt_variance: f32,
        constants: RocketConstants,
    ) -> Self {
        Self {
            orientation: initial_orientation,
            state: initial_state,
            constants,
            h: build_measurement_matrix(),
            p: todo!(),
            q: todo!(),
            r: todo!(),
        }
    }

    // what would sensor measure given current state?
    fn h(&self) -> Measurement {
        Measurement::new(
            &self.state.acceleration().into(),
            &self.state.angular_velocity().into(),
            self.state.altitude_asl(),
        )
    }

    pub fn predict(&mut self, airbrakes_ext: f32) {
        let f = central_difference_jacobian(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );

        self.state = state_transition(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        self.state
            .reset_small_angle_correction(&mut self.orientation);

        self.p = f * self.p * f.transpose() + self.q * DT;
        self.p = 0.5 * (self.p + self.p.transpose()); // keep symmetric
    }

    pub fn update(&mut self, z_rocket_frame: Measurement) {
        let z_earth_frame = Measurement::new(
            &self.orientation.inverse_transform_vector(&z_rocket_frame.acceleration()),
            &self
                .orientation
                .inverse_transform_vector(&z_rocket_frame.angular_velocity()),
            z_rocket_frame.altitude_asl(),
        );
        let y = z_earth_frame.0 - self.h().0;
        let s = self.h * self.p * self.h.transpose() + self.r;
        let k = self.p * self.h.transpose() * s.try_inverse().unwrap();

        self.state.0 += k * y;
        self.state
            .reset_small_angle_correction(&mut self.orientation);

        let i = SMatrix::<f32, { State::SIZE }, { State::SIZE }>::identity();
        self.p = (i - k * self.h) * self.p;
        self.p = 0.5 * (self.p + self.p.transpose());
    }
}
