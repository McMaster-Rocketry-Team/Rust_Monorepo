//! Airbrakes estimator.
//! Gives the airbrakes MPC altitude, vertical velocity, and tilt.
//! Only needs to be accurate after the rocket decelerates below Mach 0.8
//! post-burnout, until apogee.
//!
//! Design in one line: gyro-only tilt (no filter) + inertial dead reckoning
//! while the baro lies (transonic/supersonic shock at the static port) +
//! a drag measurement that says when the flow is subsonic again + a small
//! [altitude, vertical velocity] filter constructed fresh at that moment
//! ("born subsonic" — no state that existed during the garbage period
//! survives into the filter).
//!
//! # The lockout exit is one measurement, not a vote
//!
//! In free flight the accelerometer measures specific force, which
//! excludes gravity — so its raw magnitude IS drag/mass. Inverting
//! `a = 0.5 * rho * v^2 * Cd*A/m` therefore yields the airspeed with **no
//! integration, no attitude, and no baro**: nothing in it can drift, and
//! nothing in it can be poisoned by the very static-port error the lockout
//! exists to wait out. Measured on LC'25, swapping the barometric altitude
//! for the dead-reckoned one (the only place air density enters) moves the
//! answer by at most 0.01 Mach.
//!
//! This replaced a 2-of-3 vote over (inertial speed, deployment-filter
//! speed, baro climb-rate agreement). That vote was weaker than it looked:
//! two of its three members read the same dead reckoner, so a single
//! drifting integrator moved two votes together; the deployment filter
//! abstains for its own 12 s lockout, which outlasts the decision on a
//! Mach 2 flight; and the baro-rate member is a lie detector pointed at a
//! sensor that is known to be lying. The drag measurement subsumes all
//! three — it crosses Mach 0.8 within 0.2 s of the inertial estimate while
//! never once dipping below the threshold during the supersonic phase.
//!
//! Two conditions it depends on, both enforced by [`MachLockoutConfig`]:
//! the motor must be out (thrust tail-off briefly cancels drag, which
//! reads as a false low speed), and the brakes must be stowed (they are —
//! this gate is what opens them).
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
    /// baro is ignored from ignition until the drag vote (bounded by
    /// these timers) says the flow is subsonic again. `None` for subsonic
    /// profiles: the filter is born right after the thrust-vector
    /// alignment finishes.
    pub mach_lockout: Option<MachLockoutConfig>,

    /// Clean-airframe `Cd * A / m` (m^2/kg) — the drag vote's only
    /// parameter. Take it straight from the MPC's own `RocketParameters`
    /// via [`RocketParameters::subsonic_cda_over_mass`] so the lockout and
    /// the apogee prediction can never disagree about the airframe.
    ///
    /// It must be the SUBSONIC (brakes-stowed) value. That is what makes
    /// the vote one-sided: the true Cd is higher transonically, so the
    /// inverted speed reads high exactly while supersonic, and the vote
    /// errs toward keeping the lockout shut. Measured on LC'25 the
    /// inverted Mach peaks at 1.25 where the truth is 1.03.
    pub subsonic_cda_over_mass: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct MachLockoutConfig {
    /// Earliest possible time the rocket can be below Mach 0.8, measured
    /// from ignition detection (sim-derived). The drag vote is not
    /// consulted before this.
    ///
    /// This ALSO has to sit after the motor is out. The drag vote assumes
    /// free flight, and during thrust tail-off the residual thrust cancels
    /// part of the drag: on LC'25 the unfiltered channel dipped to an
    /// apparent Mach 0.91 at burnout while the truth was 1.14. The low
    /// pass lifts that to 1.55, but do not lean on the filter — keep
    /// `t_min_us` past `max_burn_time_us`.
    pub t_min_us: u64,
    /// Give-up time from ignition detection: at this point the filter is
    /// born from the baro regardless of the vote (sim-derived, must end
    /// well before the earliest possible apogee).
    pub t_max_us: u64,
}
