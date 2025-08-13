// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]

// use java_bindgen::prelude::*;

mod fmt;

mod controller;
mod state_estimator;
mod state_estimator2;
mod utils;

pub use state_estimator2::{FlightProfile, Measurement, RocketState, RocketStateEstimator};
pub use controller::{AirBrakesMPC, RocketParameters};
pub use utils::{approximate_air_density, approximate_speed_of_sound, lerp};

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
    pub side_reference_area: f32,
}
