use nalgebra::{SMatrix, UnitQuaternion, Vector3};

use crate::{
    RocketConstants,
    state_estimator::{DT, Measurement},
    utils::{approximate_air_density, lerp},
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

    // calculate drag coefficient
    let forward_cd = lerp(airbrakes_extention, state.drag_coefficients().as_slice());
    let cd = Vector3::new(constants.side_cd, constants.side_cd, forward_cd);
    let reference_area = Vector3::new(
        constants.side_reference_area,
        constants.side_reference_area,
        constants.front_reference_area,
    );

    let d_acc_world_frame = calculate_acc_world_frame_derivative(
        &true_orientation,
        air_density,
        &wind_vel_rocket_frame,
        &state.velocity().into(),
        &cd,
        &reference_area,
        constants,
    );
    let next_acc_world_frame = state.acceleration() + d_acc_world_frame * DT;

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
        &next_acc_world_frame,
        &(state.velocity() + state.acceleration() * DT),
        &next_angular_velocity_world_frame,
        state.altitude_asl() + state.velocity().z * DT,
        state.sideways_moment_co(),
        &state.drag_coefficients(),
    )
}

#[inline]
fn calculate_acc_world_frame_derivative(
    true_orientation: &UnitQuaternion<f32>,
    air_density: f32,
    wind_vel_rocket_frame: &Vector3<f32>,
    velocity_world_frame: &Vector3<f32>,
    cd: &Vector3<f32>,
    reference_area: &Vector3<f32>,
    constants: &RocketConstants,
) -> Vector3<f32> {
    let delta_time = 0.001f32;

    let acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
        * wind_vel_rocket_frame
            .component_mul(&wind_vel_rocket_frame.abs())
            .component_mul(cd)
            .component_mul(reference_area);
    let mut acc_world_frame = true_orientation.transform_vector(&acc_rocket_frame);
    acc_world_frame.z -= 9.81;

    let next_velocity = velocity_world_frame + acc_world_frame * delta_time;
    let next_wind_vel_rocket_frame = -true_orientation.inverse_transform_vector(&next_velocity);
    let next_acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
        * next_wind_vel_rocket_frame
            .component_mul(&next_wind_vel_rocket_frame.abs())
            .component_mul(cd)
            .component_mul(reference_area);
    let mut next_acc_world_frame = true_orientation.transform_vector(&next_acc_rocket_frame);
    next_acc_world_frame.z -= 9.81;

    // ??? 1.14 is a magic number
    (next_acc_world_frame - acc_world_frame) / delta_time * 1.14
}

pub fn central_difference_jacobian(
    airbrakes_ext: f32,
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> SMatrix<f32, { State::SIZE }, { State::SIZE }> {
    let x0 = state.0;

    let mut j_mat = SMatrix::<f32, { State::SIZE }, { State::SIZE }>::zeros();

    for j in 0..x0.len() {
        // TODO tune
        let delta = match j {
            0..3 => 0.1f32.to_radians(),
            3..6 => 0.1f32,
            6..9 => 0.1f32.max(x0[j].abs() * 0.001),
            9..12 => 1f32.to_radians(),
            12 => 0.5f32,
            13 => 0.001f32,
            14..18 => 0.01f32,
            _ => 0f32,
        };

        // x+δ
        let mut x_plus = x0;
        x_plus[j] += delta;
        let f_plus = state_transition(airbrakes_ext, orientation, &State(x_plus), constants);
        let f_plus_vec = f_plus.0;

        // x-δ
        let mut x_minus = x0;
        x_minus[j] -= delta;
        let f_minus = state_transition(airbrakes_ext, orientation, &State(x_minus), constants);
        let f_minus_vec = f_minus.0;

        // central difference: (f+ − f−) / (2δ)
        let column = (f_plus_vec - f_minus_vec) / (2.0 * delta);
        j_mat.set_column(j, &column);
    }

    j_mat
}

pub fn build_measurement_matrix() -> SMatrix<f32, { Measurement::SIZE }, { State::SIZE }> {
    let mut h = SMatrix::zeros();

    // ---- acceleration rows (rows 0-2) ----
    h[(0, 3)] = 1.0; // ax   ← state[3]
    h[(1, 4)] = 1.0; // ay   ← state[4]
    h[(2, 5)] = 1.0; // az   ← state[5]

    // ---- angular-rate rows (rows 3-5) ----
    h[(3, 9)] = 1.0; // ωx   ← state[9]
    h[(4, 10)] = 1.0; // ωy   ← state[10]
    h[(5, 11)] = 1.0; // ωz   ← state[11]

    // ---- altitude row (row 6) ----
    h[(6, 12)] = 1.0; // h    ← state[12]

    h
}
