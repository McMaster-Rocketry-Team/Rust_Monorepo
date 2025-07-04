use nalgebra::{
    AbstractRotation, Const, Matrix, Quaternion, SVector, SVectorView, Unit, UnitQuaternion,
    Vector3, ViewStorage,
};

pub struct RocketState(pub SVector<f32, 15>);

pub struct StatePropagationConstants {
    pub dt: f32,
    pub launch_site_altitude_asl: f32,
    pub side_cd: f32,
    pub moment_damping_coefficient: f32,
    pub moment_damping_coefficient_side: f32,
}

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

fn signed_square(f: f32) -> f32 {
    let sign = f.signum();
    f * f * sign
}

pub fn propagate_state(
    constants: &StatePropagationConstants,
    mut orientation: UnitQuaternion<f32>,
    RocketState(state): &RocketState,
    airbrakes_extention: f32, // 0-1
) -> RocketState {
    let delta_orientation = state.fixed_view::<3, 1>(0, 0);
    // velocity relative to wind
    let velocity = state.fixed_view::<3, 1>(3, 0);
    let angular_velocity = state.fixed_view::<3, 1>(6, 0);
    let altitude = state[9];
    let side_ways_moment_coefficient = state[10];
    let drag_coefficients = state.fixed_view::<4, 1>(11, 0);
    let (air_density, _) = approximate_air_density(constants.launch_site_altitude_asl + altitude);

    // apply small angle correction to the orientation quaternion
    let delta_orientation_quaternion: Quaternion<f32> =
        Quaternion::from_parts(1.0, delta_orientation / 2.0);
    let orientation_inner = orientation.as_mut_unchecked();
    *orientation_inner = *orientation_inner * delta_orientation_quaternion;
    orientation.renormalize();
    let orientation_inv = orientation.inverse();

    let wind_velocity_rocket_frame = -orientation.transform_vector(&velocity.into());

    // calculate drag coefficient
    let forward_cd = lerp(airbrakes_extention, drag_coefficients.as_slice());
    let cd = Vector3::new(constants.side_cd, constants.side_cd, forward_cd);

    // calculate linear acceleration
    let acceleration_rocket_frame = 0.5f32
        * air_density
        * wind_velocity_rocket_frame
            .map(signed_square)
            .component_mul(&cd);
    let mut acceleration_world_frame = orientation_inv.transform_vector(&acceleration_rocket_frame);
    acceleration_world_frame.z -= 9.81;

    // calculate angular acceleration
    let mut angular_acceleration_rocket_frame = Vector3::<f32>::zeros();
    angular_acceleration_rocket_frame.x = 0.5f32
        * air_density
        * signed_square(wind_velocity_rocket_frame.y)
        * side_ways_moment_coefficient;
    angular_acceleration_rocket_frame.y = -0.5f32
        * air_density
        * signed_square(wind_velocity_rocket_frame.x)
        * side_ways_moment_coefficient;

    let damping_coefficients = Vector3::new(
        constants.moment_damping_coefficient_side,
        constants.moment_damping_coefficient_side,
        constants.moment_damping_coefficient,
    );
    angular_acceleration_rocket_frame += -0.5f32
        * air_density
        * angular_velocity
            .map(signed_square)
            .component_mul(&damping_coefficients);
    let angular_acceleration_world_frame =
        orientation_inv.transform_vector(&angular_acceleration_rocket_frame);

    // calculate new state
    let mut new_state = state.clone();
    // update velocity
    let mut velocity = new_state.fixed_view_mut::<3, 1>(3, 0);
    velocity += acceleration_world_frame * constants.dt;

    // update altitude
    new_state[9] += velocity.z * constants.dt;

    // update angular velocity
    let mut angular_velocity = new_state.fixed_view_mut::<3, 1>(6, 0);
    angular_velocity += angular_acceleration_world_frame * constants.dt;

    // update delta_orientation
    let temp = angular_velocity * constants.dt;
    let mut delta_orientation = new_state.fixed_view_mut::<3, 1>(0, 0);
    delta_orientation += temp;

    RocketState(new_state)
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;

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
}
