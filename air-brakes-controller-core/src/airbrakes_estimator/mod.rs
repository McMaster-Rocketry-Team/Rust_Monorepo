//! Phase B v2 airbrakes estimator (see ESTIMATOR_REWORK_PLAN.md in the VLF5
//! repo). Gives the airbrakes MPC altitude, vertical velocity, and tilt.
//! Only needs to be accurate after the rocket decelerates below Mach 0.8
//! post-burnout, until apogee.
//!
//! Design in one line: gyro-only tilt (no filter) + inertial dead reckoning
//! while the baro lies (transonic/supersonic shock at the static port) +
//! a 2-of-3 vote that the baro is honest again + a small [altitude,
//! vertical velocity] filter constructed fresh at that moment ("born
//! subsonic" — no state that existed during the garbage period survives
//! into the filter).
//!
//! Every integration step uses the measured time between samples (the
//! `Measurement` carries a timestamp) — nothing assumes a perfect sample
//! rate. The flight log that motivated this had 104 ms sensor stalls.

use nalgebra::{SVector, Vector3};

mod dead_reckoner;
mod estimator;
#[cfg(test)]
mod tests;
mod vertical_kf;
pub(crate) mod welford;

pub use estimator::AirbrakesEstimator;

/// Nominal sample rate, used ONLY to size buffers and pick nominal window
/// lengths. All integration uses measured per-sample dt.
pub(crate) const NOMINAL_SAMPLES_PER_S: usize = 416;
pub(crate) const NOMINAL_DT: f32 = 1f32 / (NOMINAL_SAMPLES_PER_S as f32);
/// Per-sample dt clamp: a gap longer than this is integrated as this long
/// (protects the integrators from a bogus timestamp jump). Long enough to
/// integrate honestly through the measured 104 ms stalls.
pub(crate) const MAX_DT_S: f32 = 0.25;

/// One timestamped IMU+baro sample in the avionics (IMU chip) frame:
/// accelerometer specific force (m/s^2), angular velocity (rad/s), baro
/// altitude ASL (m). Acc and gyro must share one consistent right-handed
/// frame; the estimator self-calibrates the mounting orientation on the
/// pad, so no per-board axis configuration is needed.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct Measurement {
    pub timestamp_us: u64,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    data: SVector<f32, 7>,
}

impl Measurement {
    pub fn new(
        timestamp_us: u64,
        acceleration: &Vector3<f32>,
        angular_velocity: &Vector3<f32>,
        altitude_asl: f32,
    ) -> Self {
        let mut data = SVector::<f32, 7>::zeros();
        data.fixed_view_mut::<3, 1>(0, 0).copy_from(acceleration);
        data.fixed_view_mut::<3, 1>(3, 0).copy_from(angular_velocity);
        data[6] = altitude_asl;
        Self { timestamp_us, data }
    }

    pub fn acceleration(&self) -> Vector3<f32> {
        self.data.fixed_view::<3, 1>(0, 0).into()
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        self.data.fixed_view::<3, 1>(3, 0).into()
    }

    pub fn altitude_asl(&self) -> f32 {
        self.data[6]
    }
}

/// Airbrakes estimator configuration. All numbers are per-airframe /
/// per-motor and come from the flight simulation or prior flight data.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct AirbrakesConfig {
    /// Low-passed |accel| above this latches ignition detection (m/s^2).
    /// ~4 g works for most motors: well above pad handling and wind, well
    /// below liftoff thrust.
    pub ignition_detection_acc_threshold: f32,

    /// `Some` for flights that go near or above the speed of sound: the
    /// baro is ignored from ignition until the 2-of-3 vote (bounded by
    /// these timers) says it is honest again. `None` for subsonic
    /// profiles: the filter is born right after the thrust-vector
    /// alignment finishes.
    pub mach_lockout: Option<MachLockoutConfig>,

    /// Fixed static-port coefficient c-hat: the baro reads HIGH by
    /// roughly c * airspeed^2 meters (shock/position error at the static
    /// port grows with dynamic pressure). Per-airframe constant from CFD,
    /// sim, or a prior flight of the same airframe (LC'25 measured
    /// +0.7e-3). Applied to every baro reading this estimator consumes.
    /// Units: m / (m/s)^2.
    pub baro_port_coefficient: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct MachLockoutConfig {
    /// Earliest possible time the rocket can be below Mach 0.75, measured
    /// from ignition detection (sim-derived). The vote is not consulted
    /// before this.
    pub t_min_us: u64,
    /// Give-up time from ignition detection: at this point the filter is
    /// born from the baro regardless of the vote (sim-derived, must end
    /// well before the earliest possible apogee).
    pub t_max_us: u64,
}
