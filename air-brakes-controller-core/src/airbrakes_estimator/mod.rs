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
//! # The lockout exit is one measurement
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
//! It crosses Mach 0.8 within 0.2 s of the inertial estimate while never
//! once dipping below the threshold during the supersonic phase.
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

use crate::controller::RocketParameters;

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
    /// baro is ignored from ignition until the drag check (bounded by
    /// these timers) says the flow is subsonic again. `None` for subsonic
    /// profiles: the filter is born right after the thrust-vector
    /// alignment finishes.
    pub mach_lockout: Option<MachLockoutConfig>,

    /// The airframe — the same value the MPC flies on.
    ///
    /// The drag check needs `Cd * A / m`, and it derives that here rather
    /// than taking it as a number, so the lockout and the apogee
    /// prediction cannot be given different airframes. It reads `cd[0]`,
    /// the brakes-stowed entry, and that is what makes the check one-sided:
    /// the true Cd is higher transonically, so the inverted speed reads
    /// high exactly while supersonic and the check errs toward keeping the
    /// lockout shut. Measured on LC'25 the inverted Mach peaks at 1.25
    /// where the truth is 1.03.
    pub rocket: RocketParameters,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
/// Bounds on when the drag check is allowed to decide, both measured from
/// **this estimator's own accelerometer ignition detection**.
///
/// Note the clock: [`FlightProfile::mach_lockout_duration_us`] runs from the
/// DEPLOYMENT estimator's baro ignition detection instead, which lags. The
/// two lockouts are independent — different subsystems, different sensors,
/// different thresholds — and equal values in a config are coincidence, not
/// a relationship. Changing one does not imply changing the other.
///
/// [`FlightProfile::mach_lockout_duration_us`]: crate::FlightProfile::mach_lockout_duration_us
pub struct MachLockoutConfig {
    /// Earliest the rocket could possibly be below Mach 0.8; the drag check
    /// is not consulted before this.
    ///
    /// From the flight sim: the earliest simulated time below Mach 0.8,
    /// measured from ignition detection. Erring early is unsafe (the check
    /// gets to speak while still supersonic), erring late only costs
    /// control window.
    ///
    /// It does not have to be placed after burnout by hand: the estimator
    /// latches burnout itself from the sign of the axial specific force and
    /// refuses to birth the filter before then, by either path. Set this
    /// purely from the sim's earliest subsonic time.
    pub earliest_subsonic_after_ignition_us: u32,
    /// Give-up time: at this point the vertical filter is born from the
    /// baro regardless of what the drag check says.
    ///
    /// From the flight sim: the latest plausible time below Mach 0.8 plus
    /// margin, and it must end well (>5 s) before the EARLIEST simulated
    /// apogee — a forced birth after apogee leaves the airbrakes no window
    /// at all.
    ///
    /// This backstop is still subject to the burnout latch: it covers a
    /// drag model wrong enough that the check never passes (the axial sign
    /// test does not depend on Cd, so the latch still fires), not an
    /// accelerometer too dead to show deceleration at all.
    pub force_birth_after_ignition_us: u32,
}
