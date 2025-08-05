use approx::assert_relative_eq;
use nalgebra::{UnitQuaternion, UnitVector3, Vector3, Vector4};

use crate::{
    RocketConstants,
    state_estimator::{
        DT,
        ascent_fusion::mekf::{state::State, state_propagation::state_transition},
    },
    tests::init_logger,
};

const ROCKET_CONSTANTS: RocketConstants = RocketConstants {
    initial_front_cd: [0.5, 0.5, 0.5, 0.5],
    initial_sideways_moment_co: 0.3,
    side_cd: 0.5,
    burn_out_mass: 17.625,
    moment_of_inertia: 11.11,
    front_reference_area: 0.3,
    side_reference_area: 0.3,
};

#[test]
fn test_small_angle_correction() {
    init_logger();

    let orientation = UnitQuaternion::identity();
    let state = State::new(
        &Vector3::new(1.0f32.to_radians(), 0.0, 0.0),
        &Vector3::zeros(),
        &Vector3::zeros(),
        &(Vector3::new(1.0f32.to_radians(), 0.0, 0.0) / DT),
        0.0,
        0.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    let mut new_state = state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);

    let new_sac_deg = new_state.small_angle_correction().map(|r| r.to_degrees());
    log_info!("new sac deg: {}", new_sac_deg);
    assert_relative_eq!(new_sac_deg.x, 2.0);

    let new_orientation = new_state.reset_small_angle_correction(&orientation);
    log_info!("new_orientation: {}", new_orientation);
    let (axis, angle) = new_orientation.axis_angle().unwrap();
    assert_relative_eq!(angle.to_degrees(), 2.0);
    assert_relative_eq!(axis.x, 1.0);
}

#[test]
fn test_wind_vel() {
    init_logger();

    let orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
        30f32.to_radians(),
    );
    let state = State::new(
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::new(0.0, 1.0, 0.0),
        &Vector3::zeros(),
        0.0,
        0.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);
    // inside the state_transition function:
    // wind_vel_rocket_frame.x = 0
    // wind_vel_rocket_frame.y < 0
    // wind_vel_rocket_frame.z > 0
}


#[test]
fn test_acc_world_frame() {
    init_logger();

    let orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
        30f32.to_radians(),
    );
    let state = State::new(
        &Vector3::zeros(),
        &Vector3::zeros(),
        // velocity towards the rocket nose
        &Vector3::new(0.0, -1.0 * 10.0, 3f32.sqrt() * 10.0),
        &Vector3::zeros(),
        0.0,
        0.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);
    // inside the calculate_acc_world_frame_derivative function:
    // acc_rocket_frame.x = 0
    // acc_rocket_frame.y = 0
    // acc_rocket_frame.z < 0
    //
    // acc_world_frame.x (before adding gravity) = 0
    // acc_world_frame.y (before adding gravity) > 0
    // acc_world_frame.z (before adding gravity) < 0
}

#[test]
fn test_angular_acc() {
    init_logger();

    let orientation = UnitQuaternion::identity();
    let state = State::new(
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::new(10.0, 10.0, 0.0),
        &Vector3::zeros(),
        0.0,
        1.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);
    // inside the state_transition function:
    // angular_acceleration_rocket_frame.x < 0
    // angular_acceleration_rocket_frame.y > 0
    // angular_acceleration_rocket_frame.z = 0
}

#[test]
fn test_angular_acc_world_frame() {
    init_logger();

    let orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
        90f32.to_radians(),
    );
    let state = State::new(
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::new(10.0, 10.0, 0.0),
        &Vector3::zeros(),
        0.0,
        1.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);
    // inside the state_transition function:
    // angular_acceleration_world_frame.x < 0
    // angular_acceleration_world_frame.y > 0
    // angular_acceleration_world_frame.z = 0
}

#[test]
fn test_angular_vel() {
    init_logger();

    let orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
        90f32.to_radians(),
    );
    let state = State::new(
        &Vector3::zeros(),
        &Vector3::zeros(),
        &Vector3::zeros(),
        &(Vector3::new(10.0f32.to_radians(), 0.0, 0.0) / DT),
        0.0,
        1.0,
        &Vector4::new(0.5, 0.5, 0.5, 0.5),
    );

    let mut new_state = state_transition(0.0, &orientation, &state, &ROCKET_CONSTANTS);

    let new_sac_deg = new_state.small_angle_correction().map(|r| r.to_degrees());
    log_info!("new sac deg: {}", new_sac_deg);
    assert_relative_eq!(new_sac_deg.y, -10.0);

    let new_orientation = new_state.reset_small_angle_correction(&orientation);
    // should print [0.061628412, -0.061628412, 0.70441604, 0.70441604]
    log_info!("new_orientation: {:?}", new_orientation);
}