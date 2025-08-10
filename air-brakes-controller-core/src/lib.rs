// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]

// use java_bindgen::prelude::*;

mod fmt;
mod state_estimator;
pub mod state_estimator2;
mod utils;

pub use state_estimator::RocketStateEstimator;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
pub struct RocketConstants {
    // front cd at 0%, 33%, 66% 100% air brakes
    pub initial_front_cd: [f32; 4],
    pub initial_sideways_moment_co: f32,
    pub side_cd: f32,
    pub burn_out_mass: f32,
    pub moment_of_inertia: f32,
    pub front_reference_area: f32,
    pub side_reference_area: f32
}

// #[derive(Default, JavaClass)]
// struct OpenRocketPostStepInput {
//     acc_x: f64,
//     acc_y: f64,
//     acc_z: f64,
//     gyro_x: f64,
//     gyro_y: f64,
//     gyro_z: f64,
//     pressure: f64,
// }

// #[derive(Default, JavaClass)]
// struct OpenRocketPostStepOutput {
//     est_altitude: f32,
//     est_aoa: f32,
//     est_horizontal_velocity: f32,
//     est_tilt: f32,
//     est_cd: f32,
//     est_side_moment_co: f32,
//     extension_percentage: f32,
//     ap_residue: f32,
// }

// #[java_bindgen]
// fn openrocket_post_step(input: OpenRocketPostStepInput) -> JResult<OpenRocketPostStepOutput> {
//     Ok(OpenRocketPostStepOutput {
//         est_altitude: 0.0,
//         est_aoa: 1.0,
//         est_horizontal_velocity: 2.0,
//         est_tilt: 3.0,
//         est_cd: 4.0,
//         est_side_moment_co: 5.0,
//         extension_percentage: 6.0,
//         ap_residue: 7.0,
//     })
// }
