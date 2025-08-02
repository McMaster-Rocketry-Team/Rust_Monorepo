use nalgebra::{SMatrix, UnitQuaternion};

use crate::mekf::{state_propagation::{calculate_state_derivative, central_difference_jacobian, ROCKET_MEASUREMENT_SIZE}, State, RocketConstants};

const DT: f32 = 1f32 / 200f32;

pub struct RocketMEKF {
    orientation: UnitQuaternion<f32>,
    state: State,
    constants: RocketConstants,

    /// covariance P_k|k
    P: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// process-noise covariance Q
    Q: SMatrix<f32, { State::SIZE }, { State::SIZE }>,

    /// measurement-noise covariance R
    R: SMatrix<f32, { ROCKET_MEASUREMENT_SIZE }, { ROCKET_MEASUREMENT_SIZE }>,
}

impl RocketMEKF {
    pub fn new(initial_orientation: UnitQuaternion<f32>,initial_state: State, constants: RocketConstants) -> Self {
        Self {
            orientation:initial_orientation,
            state: initial_state,
            constants,
            P: todo!(),
            Q: todo!(),
            R: todo!(),
        }
    }

    pub fn predict(&mut self, airbrakes_ext: f32) {
        let mut Fk = central_difference_jacobian(airbrakes_ext, &self.orientation, &self.state, &self.constants);
        Fk.iter_mut().for_each(|v| *v *= DT);
        
        let state_derivative = calculate_state_derivative(airbrakes_ext, &self.orientation, &self.state, &self.constants);
        self.state.add_derivative(&state_derivative, DT);
        self.state.reset_small_angle_correction(&mut self.orientation);

        self.P = &Fk * &self.P * Fk.transpose() + self.Q * DT;
        self.P = 0.5 * (&self.P + self.P.transpose()); // keep symmetric
    }
}
