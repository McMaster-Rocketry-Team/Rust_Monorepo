// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]
// `utils::sqrt` reaches the FPU's VSQRT instruction through the intrinsic;
// `core` has no other route to it, and `libm::sqrtf` runs a 714-cycle
// software algorithm on this target instead. Only the no_std build needs it —
// under std the same function is `f32::sqrt`.
#![cfg_attr(
    not(any(test, feature = "std")),
    feature(core_intrinsics),
    allow(internal_features)
)]

// use java_bindgen::prelude::*;

mod fmt;

pub mod airbrakes_estimator;
pub mod baro_gate;
pub mod baro_state_estimator;
mod controller;
pub mod flight_estimators;
pub mod ignition_detector;
mod utils;

pub use baro_state_estimator::{
    DeploymentProfile, FlightProfile, RocketState, RocketStateEstimator,
};
pub use baro_gate::BaroGateOutcome;
pub use ignition_detector::IgnitionDetector;
pub use airbrakes_estimator::ImuSample;
pub use flight_estimators::{
    AirbrakesLogSample, AirbrakesMPCStates, EstimatorLogSample, FlightConfig, FlightEstimators,
};
pub use controller::{AirBrakesMPC, MpcSolution, RocketParameters};
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
