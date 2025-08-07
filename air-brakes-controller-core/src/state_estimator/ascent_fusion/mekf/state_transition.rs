use nalgebra::{SMatrix, UnitQuaternion, Vector3};

use crate::{
    RocketConstants,
    state_estimator::{DT, Measurement},
    utils::approximate_air_density,
};

use super::state::State;

pub fn state_transition(
    airbrakes_extention: f32, // 0-1
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> State {
    let air_density = approximate_air_density(state.altitude_asl());
    let delta_orientation = UnitQuaternion::from_scaled_axis(state.small_angle_correction());
    let true_orientation = orientation * delta_orientation;

    let wind_vel_rocket_frame =
        -true_orientation.inverse_transform_vector(&state.velocity().into());

    let mut angular_acceleration_rocket_frame = Vector3::<f32>::zeros();
    angular_acceleration_rocket_frame.x = 0.5 * air_density / constants.moment_of_inertia
        * constants.side_reference_area
        * wind_vel_rocket_frame.y
        * wind_vel_rocket_frame.y.abs()
        * state.sideways_moment_co();
    angular_acceleration_rocket_frame.y = -0.5 * air_density / constants.moment_of_inertia
        * constants.side_reference_area
        * wind_vel_rocket_frame.x
        * wind_vel_rocket_frame.x.abs()
        * state.sideways_moment_co();

    let angular_acceleration_world_frame =
        true_orientation.transform_vector(&angular_acceleration_rocket_frame);
    let next_angular_velocity_world_frame =
        state.angular_velocity() + angular_acceleration_world_frame * DT;

    let angular_velocity_rocket_frame =
        true_orientation.inverse_transform_vector(&state.angular_velocity().into());
    let next_small_angle_correction =
        state.small_angle_correction() + angular_velocity_rocket_frame * DT;

    State::new(
        &next_small_angle_correction,
        &(state.velocity()
            + state.expected_acceleration(airbrakes_extention, orientation, constants) * DT),
        &next_angular_velocity_world_frame,
        state.altitude_asl() + state.velocity().z * DT,
        state.sideways_moment_co(),
        &state.drag_coefficients(),
    )
}

pub fn state_transition_jacobian(
    airbrakes_ext: f32,
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> SMatrix<f32, { State::SIZE }, { State::SIZE }> {
    super::jacobian::central_difference_jacobian(&state.0, |v| {
        state_transition(airbrakes_ext, orientation, &State(*v), constants).0
    })
}