use nalgebra::{Quaternion, SMatrix, SVector, UnitQuaternion, Vector3, Vector4};

use crate::{RocketConstants, state_estimator::Measurement};

use super::state::{Derivative, State};

/// returns air density (kg/m^3) and speed of sound (m/s) at altitude (m)
/// approximated using a linear function from 0m and 3000m data from standard atmosphere model
pub fn approximate_air_density(altitude_asl: f32) -> (f32, f32) {
    let air_density = 1.225 - altitude_asl * 0.0001053;
    let speed_of_sound = 340.29 - altitude_asl * 0.003903;

    (air_density, speed_of_sound)
}

fn lerp(
    t: f32, // 0-1
    drag_coefficients: &[f32],
) -> f32 {
    let len = drag_coefficients.len();
    let spacing = 1.0f32 / ((len - 1) as f32);

    let mut i = (t / spacing) as usize;
    if i > len - 2 {
        i = len - 2;
    }

    let t = (t - spacing * (i as f32)) * (len - 1) as f32;
    (1.0 - t) * drag_coefficients[i] + t * drag_coefficients[i + 1]
}

pub fn calculate_state_derivative(
    airbrakes_extention: f32, // 0-1
    orientation: &UnitQuaternion<f32>,
    state: &State,
    constants: &RocketConstants,
) -> Derivative<State> {
    let (air_density, _) = approximate_air_density(state.altitude_asl());
    let delta_orientation = UnitQuaternion::from_quaternion(Quaternion::from_parts(
        1.0,
        state.small_angle_correction() / 2.0,
    ));
    let true_orientation = delta_orientation * orientation;

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

    // FIXME try to calculate d_acc (jerk) analytically gives the wrong result for some reason
    // let d_acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
    //     * 2.0
    //     * wind_vel_rocket_frame
    //         .abs()
    //         .component_mul(&cd)
    //         .component_mul(&reference_area);

    let d_acc_world_frame = calculate_acc_world_frame_derivative(
        &true_orientation,
        air_density,
        &wind_vel_rocket_frame,
        &state.velocity().into(),
        &cd,
        &reference_area,
        constants,
    );

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

    let angular_velocity_rocket_frame =
        true_orientation.inverse_transform_vector(&state.angular_velocity().into());

    Derivative(State::new(
        &angular_velocity_rocket_frame,
        &d_acc_world_frame,
        &state.acceleration().into(),
        &angular_acceleration_world_frame,
        state.velocity().z,
        0.0,
        &Vector4::zeros(),
    ))
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
        let f_plus =
            calculate_state_derivative(airbrakes_ext, orientation, &State(x_plus), constants);
        let f_plus_vec = f_plus.0.0;

        // x-δ
        let mut x_minus = x0;
        x_minus[j] -= delta;
        let f_minus =
            calculate_state_derivative(airbrakes_ext, orientation, &State(x_minus), constants);
        let f_minus_vec = f_minus.0.0;

        // central difference: (f+ − f−) / (2δ)
        let column = (f_plus_vec - f_minus_vec) / (2.0 * delta);
        j_mat.set_column(j, &column);
    }

    j_mat
}

pub struct RocketMeasurement(pub SVector<f32, { State::SIZE }>);

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

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use nalgebra::UnitVector3;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn lerp_test() {
        assert_relative_eq!(
            lerp(-1f32 / 3.0, &[0.0, 1.0, 2.0, 3.0]),
            -1.0,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(0.0f32, &[0.0, 1.0, 2.0, 3.0]), 0.0, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(0.16666666f32, &[0.0, 1.0, 2.0, 3.0]),
            0.5,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(0.5f32, &[0.0, 1.0, 2.0, 3.0]), 1.5, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(0.83333333f32, &[0.0, 1.0, 2.0, 3.0]),
            2.5,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(1.0f32, &[0.0, 1.0, 2.0, 3.0]), 3.0, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(1.0f32 + 1.0 / 3.0, &[0.0, 1.0, 2.0, 3.0]),
            4.0,
            epsilon = 0.0001
        );
    }

    #[test]
    fn small_angle_correction_test() {
        init_logger();

        fn euler_angles_to_degrees(radians: (f32, f32, f32)) -> [f32; 3] {
            [
                radians.0.to_degrees(),
                radians.1.to_degrees(),
                radians.2.to_degrees(),
            ]
        }

        let small_angle = Vector3::new(0f32.to_radians(), 0f32.to_radians(), 1f32.to_radians());
        let delta_orientation_quaternion: UnitQuaternion<f32> =
            UnitQuaternion::from_quaternion(Quaternion::from_parts(1.0, -small_angle / 2.0));

        let mut orientation = UnitQuaternion::<f32>::identity();
        orientation = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_normalize(Vector3::new(1f32, 0f32, 0f32)),
            90f32.to_radians(),
        );
        log_info!(
            "initial orientation: {:?}",
            euler_angles_to_degrees(orientation.euler_angles())
        );
        orientation = delta_orientation_quaternion * orientation;

        log_info!(
            "rotated orientation: {:?}",
            euler_angles_to_degrees(orientation.euler_angles())
        );
    }
}
