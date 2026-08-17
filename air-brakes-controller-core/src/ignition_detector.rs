//! The accelerometer ignition detector, shared by both estimator halves.
//!
//! Both halves have to answer the same question — *has the motor lit?* — and
//! until 2026-08-17 each answered it with its own copy of the same three
//! lines: a one-pole low pass, a magnitude threshold, a sustain. The copies
//! had drifted apart in every parameter (10 Hz vs 5 Hz, sustain vs no
//! sustain, 4 g vs 8 g), and the drift was not deliberate — the airbrakes
//! copy simply never grew the sustain the pyro copy has, so it latched on
//! the first sample over threshold. A 10 ms knock on the rail was enough.
//!
//! One implementation, two instances. Sharing the *type* is what makes the
//! two halves provably agree about what ignition means; sharing an
//! *instance* would be wrong, because the halves are not allowed to detect
//! at the same instant — see [`IgnitionDetector::update`].
//!
//! The threshold stays out here, in each half's config, because it is the
//! one parameter that is genuinely per-airframe: it is sized against the
//! motor's thrust curve, and a bench profile whose scripted motor reads
//! 9.15 g cannot use the number a 14 g O-motor wants.

use nalgebra::Vector3;

/// Time constant of the low pass in front of the threshold — a 10 Hz corner
/// (`1 / 2*pi*10`).
///
/// This is the airbrakes half's old value, not the pyro half's 5 Hz. The
/// 5 Hz was justified as buying quiet for a detector that starts a Mach
/// lockout, at the cost of a few tens of milliseconds — but quiet against a
/// transient is what [`SUSTAIN_S`] buys, two orders of magnitude more of it,
/// and the low pass was doing that job badly by comparison. With the sustain
/// in place the corner only has to keep sensor noise off the threshold, and
/// 10 Hz does that against a 0.04 m/s^2 pad noise floor with three orders of
/// magnitude to spare. So the merge takes the faster corner and the longer
/// sustain, and is strictly better than either copy on both counts.
const LP_TAU_S: f32 = 0.0159;

/// How long the low-passed magnitude must stay continuously above the
/// threshold before ignition latches.
///
/// This is what actually rejects a knock on the rail. The low pass alone
/// rejects a transient only in proportion to how much it attenuates it: a
/// 10 ms impulse through `LP_TAU_S` still reaches about half its amplitude,
/// so a detector with no sustain latches on any transient of roughly twice
/// the threshold, which a dropped rocket or a hand on the rail can supply.
/// A sustain rejects it outright, and it costs 0.1 s of a detection that
/// wins ~1.1 s over the barometric detector it replaced.
///
/// Every other latch in both estimators is sustained the same way.
const SUSTAIN_S: f32 = 0.1;

/// One half's view of "has the motor lit?".
#[derive(Debug, Clone, Copy)]
pub struct IgnitionDetector {
    /// Low-passed accelerometer. `None` until the first sample that carries
    /// one.
    acc_lp: Option<Vector3<f32>>,
    /// How long the low-passed magnitude has been continuously above the
    /// threshold, in seconds of measured time.
    sustain_s: f32,
}

impl IgnitionDetector {
    pub const fn new() -> Self {
        Self {
            acc_lp: None,
            sustain_s: 0.0,
        }
    }

    /// Advance the detector by one sample and report whether ignition has
    /// latched. `threshold` is the specific-force magnitude in m/s^2.
    ///
    /// Call this on EVERY sample, so the low pass and the sustain are
    /// already warm when the motor lights, and consult the result only
    /// where the caller is allowed to act on it. The two halves are not
    /// allowed to act at the same instant: the airbrakes half refuses to
    /// detect ignition before its pad calibration completes, and a board
    /// powered up seconds before launch would fire no pyros at all if the
    /// pyro half waited on that. Hence two instances, each gated by its own
    /// half's preconditions.
    ///
    /// A sample without an IMU reading leaves the filter and the sustain
    /// exactly where they were rather than resetting them: a one-sample SPI
    /// glitch mid-boost is not evidence that the motor stopped.
    pub fn update(&mut self, acc: Option<Vector3<f32>>, dt: f32, threshold: f32) -> bool {
        let Some(acc) = acc else {
            return false;
        };

        // One pole on the measured dt. Clamped at alpha = 1 so a long stall
        // snaps to the sample rather than overshooting past it.
        //
        // The vector is filtered and then measured, not the other way round:
        // |accel| is rectified, so low-passing the magnitude would let
        // airframe vibration bias the channel upward toward the threshold.
        let lp = match self.acc_lp {
            Some(prev) => prev + (dt / LP_TAU_S).min(1.0) * (acc - prev),
            None => acc,
        };
        self.acc_lp = Some(lp);

        if lp.magnitude_squared() > threshold * threshold {
            self.sustain_s += dt;
        } else {
            self.sustain_s = 0.0;
        }
        self.sustain_s >= SUSTAIN_S
    }
}

impl Default for IgnitionDetector {
    fn default() -> Self {
        Self::new()
    }
}
