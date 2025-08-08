use nalgebra::{SMatrix, UnitQuaternion};

use crate::{
    state_estimator::{ascent_fusion::mekf::{jacobian::central_difference_jacobian, State}, Measurement}, RocketConstants
};

// h
pub fn measurement_model(
    airbrakes_extention: f32, // 0-1
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> Measurement {
    // Measurement provides specific force in earth frame (accelerometer minus gravity),
    // so we output model-predicted specific force (non-gravitational acceleration) here.
    Measurement::new(
        &state
            .expected_acceleration(airbrakes_extention, orientation, constants)
            .into(),
        &state.angular_velocity().into(),
        state.altitude_asl(),
    )
}

pub fn measurement_model_jacobian(
    airbrakes_ext: f32,
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> SMatrix<f32, { Measurement::SIZE }, { State::SIZE }> {
    central_difference_jacobian(&state.0, |v| {
        measurement_model(airbrakes_ext, orientation, &State(*v), constants).0
    })
}
