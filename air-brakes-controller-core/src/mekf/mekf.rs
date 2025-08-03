use nalgebra::{SMatrix, UnitQuaternion};

use crate::mekf::{
    Measurement, RocketConstants, State,
    state_propagation::{
        build_measurement_matrix, calculate_state_derivative, central_difference_jacobian,
    },
};

const DT: f32 = 1f32 / 500f32;

pub struct RocketMEKF {
    orientation: UnitQuaternion<f32>,
    state: State,
    constants: RocketConstants,

    /// covariance P_k|k
    P: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// process-noise covariance Q
    Q: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// measurement-noise covariance R
    R: SMatrix<f32, { Measurement::SIZE }, { Measurement::SIZE }>,

    /// measurement matrix
    H: SMatrix<f32, { Measurement::SIZE }, { State::SIZE }>,
}

impl RocketMEKF {
    pub fn new(
        initial_orientation: UnitQuaternion<f32>,
        initial_state: State,
        constants: RocketConstants,
    ) -> Self {
        Self {
            orientation: initial_orientation,
            state: initial_state,
            constants,
            H: build_measurement_matrix(),
            P: todo!(),
            Q: todo!(),
            R: todo!(),
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
        let mut Fk = central_difference_jacobian(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        Fk.iter_mut().for_each(|v| *v *= DT);

        let state_derivative = calculate_state_derivative(
            airbrakes_ext,
            &self.orientation,
            &self.state,
            &self.constants,
        );
        self.state.add_derivative(&state_derivative, DT);
        self.state
            .reset_small_angle_correction(&mut self.orientation);

        self.P = &Fk * &self.P * Fk.transpose() + self.Q * DT;
        self.P = 0.5 * (&self.P + self.P.transpose()); // keep symmetric
    }

    pub fn update(&mut self, z: Measurement) {
        let y = z.0 - self.h().0; // innovation
        let S = self.H * &self.P * self.H.transpose() + &self.R;
        let K = self.P * self.H.transpose() * S.try_inverse().unwrap();

        self.state.0 += &K * y;
        self.P =
            (SMatrix::<f32, { State::SIZE }, { State::SIZE }>::identity() - &K * self.H) * &self.P;
        self.P = 0.5 * (&self.P + self.P.transpose());
    }
}
