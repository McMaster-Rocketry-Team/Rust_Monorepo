//! *Deployment* state machine, baro-driven.
//!
//! Detects apogee and landing from barometric altitude alone via a
//! deliberately slow (~1 s bandwidth) 2-state Kalman filter whose output is
//! trusted outright — the COTS-altimeter shape: innovation gate as bus input
//! validation, a timed Mach lockout started at ignition detection, apogee by
//! peak-drop on the filtered altitude, coasting by burn timer, and
//! "condition holds for N seconds" persistence on every transition. Accuracy
//! is explicitly not a goal (boost lag is hundreds of metres); the airbrakes
//! use a separate fast estimator. Supports single (both pyros at apogee) and
//! dual (drogue at apogee, main at altitude) deployment via [`FlightProfile`].
//!
//! **Ignition is the one decision the barometer does not make.** It is a
//! magnitude check on the raw accelerometer and nothing else
//! ([`FlightConfig::ignition_detection_acc_threshold`]): a 10 Hz low pass, a
//! threshold, and a 0.1 s sustain, implemented once in
//! [`crate::ignition_detector`] and instantiated here. It needs no pad
//! calibration, no mounting orientation, no gyro bias — and it shares the
//! *code* with the airbrakes half's detector but not the *instance*, so
//! neither half can hold the other's ignition decision hostage. The
//! threshold is not even per-half: it is one field above both of them.
//!
//! [`FlightConfig::ignition_detection_acc_threshold`]:
//!     crate::FlightConfig::ignition_detection_acc_threshold
//!
//! The barometric detector it replaced (10 m/s of filtered climb AND 15 m of
//! rise) decided the most load-bearing instant in the flight — the anchor
//! for the Mach lockout — using the very static port that was about to stop
//! telling the truth, through a filter with ~1 s of bandwidth, finishing
//! only 0.75 s ahead of it on Osiris. It also ran ~1.1 s later than the
//! accelerometer on every flight measured.
//!
//! **This estimator therefore does not detect a launch without a working
//! accelerometer.** There is no second opinion. An IMU that reads
//! successfully but reports low leaves it on the pad, and no pyro fires.
//! That is a deliberate trade: one detector that is right about the moment
//! that matters, over a second one that is specifically wrong about it.
//!
//! # What the filter is for, and when it exists
//!
//! Only for apogee and landing, and so only from the moment the barometer
//! is worth filtering:
//!
//! * **On the pad** there is no filter. The barometer's one job is the pad
//!   altitude reference, which is a plain mean of one second of readings
//!   taken a second before the rocket moves — see [`PadReference`].
//! * **Through the Mach lockout** there is no filter. It is dropped at
//!   ignition rather than frozen, so no caller can read a pre-ignition
//!   altitude out of it while the rocket is kilometres away. The raw baro
//!   is still buffered through it, for one purpose only — see
//!   [`BARO_RING_CAP`].
//! * **After the lockout** one is built from the median of the last
//!   [`BARO_RING_SPAN_S`] of readings, and runs to landing. On a subsonic
//!   profile it is built at ignition instead, from the tracked pad
//!   altitude. Neither birth reads a single raw sample: the filter defends
//!   its own state with an innovation gate afterwards, so the one number it
//!   is handed first must not be able to be a bad SPI read.
//!
//! `Option<BaroAltitudeKF>` is the whole of that rule. Absence is a fact
//! about the type rather than something every reader has to remember.

mod altitude_kf;

#[cfg(test)]
mod tests;

pub use altitude_kf::BaroAltitudeKF;

use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use heapless::Deque;
use nalgebra::Vector3;

use crate::ignition_detector::IgnitionDetector;

use crate::baro_gate::BaroGateOutcome;

/// Baro sample rate the KF is designed for (matches IMU ODR).
///
/// The **filter** is clocked by this and nothing else: one fixed `DT`
/// predict step per sample, forever. That is deliberate. This is the half
/// that fires the pyros, and a fixed-step filter cannot be surprised by a
/// clock — no timestamp it is handed can change its bandwidth, its gains,
/// or how far it propagates. It is meant to be dumb.
///
/// Every *duration* in this module is wall-clock instead — see
/// [`RocketStateEstimator::update`]. Those are the numbers that have to be
/// right in seconds (a 26 s Mach lockout, a 1 s drogue delay), and they
/// were measurably wrong while they were counted in samples: the part
/// actually runs at 427.02 Hz, so every one of them expired 2.65% early.
pub const SAMPLES_PER_S: usize = 416;
pub const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

/// Apogee detection: filtered altitude this far below its running maximum
/// counts as descending. Must exceed the worst transient dip a gate-leaking
/// blast can put on the slow filter (~30 m from a 25-sample 500 m offset,
/// which then decays in ~1 s — too short for the persistence window below).
const APOGEE_DROP_M: f32 = 30.0; // m
/// How long the altitude has to stay below (peak - APOGEE_DROP_M) before
/// descent is acted upon
const APOGEE_DROP_SUSTAIN_S: f32 = 0.5;
/// |KF vertical velocity| below this counts as standing still. The slow
/// filter's stationary velocity noise is ~0.012 m/s std (peaks ~0.05 m/s), so
/// this is sized by canopy-swing and post-touchdown-drift rejection, not
/// noise; descent under main (>= ~4.5 m/s) keeps the counter pinned at zero.
const LANDED_VELOCITY_THRESHOLD: f32 = 2.0; // m/s
/// How long the rocket has to stand still before it is considered landed
const LANDED_DETECTION_S: f32 = 5.0;
/// Length of one pad-reference averaging window (s of measured time). See
/// [`PadReference`] for why the windows are handed out one behind.
const PAD_WINDOW_S: f32 = 1.0;

/// Ring of recent raw baro samples kept through the Mach lockout, used for
/// exactly one thing: the state the filter is born in when the lockout ends
/// — where the airframe is and how fast it is climbing, both by median. Same
/// idiom, and the same nine picks, as the airbrakes half's `BARO_RING_CAP` /
/// `ring_median` — see [`ring_birth_state`] for why this is a copy rather
/// than a shared function, and for where the two differ.
///
/// It exists because a birth seeded from ONE raw sample is a birth that
/// inherits whatever that sample happened to be, and the exit sample is not
/// gated by anything: the old filter was dropped at ignition, so there is
/// no innovation gate left to reject it, and `peak_altitude_asl` was seeded
/// from the same reading. Measured on the Osiris O3400 sim with a single
/// bad reading on the exit sample — an SPI read decoding to pressure ~0
/// (~30 km), or merely a factor-of-2 pressure error (12854 m against an
/// honest 8359 m) — the drogue fired at 28.67 s while the airframe was
/// still climbing at +109.7 m/s, against 43.60 s nominal. Both cases: the
/// filter and the peak were born at the bad altitude, so the very next
/// honest sample read as kilometres of descent. Take the KF's force-accept
/// resync away as well and the filter never re-acquires at all — no pyros.
///
/// A median outvotes it. `BARO_RING_SPAN_S` = 0.25 s is the whole margin:
/// long enough that a transient has to hold for more than half the window
/// (5 of the 9 picks, ~0.13 s) to move the answer, short enough that the
/// half-window lag it comes with is one the ring can measure its way out of.
///
/// That lag used to be paid, and the claim here used to be that it did not
/// matter — the newborn filter's (300 m/s)^2 velocity prior would close it
/// in a fraction of a second. It closed it by *exploding*: the HIL replay
/// exits the lockout at 141 m/s, which puts the median 17.8 m below where
/// the airframe is, and the first update turned that into a published
/// vertical velocity of 2858 m/s and 3.6 m of altitude overshoot. The
/// velocity feeds only the log and the downlink and the overshoot is far
/// under `APOGEE_DROP_M`, so nothing was mis-fired — but the lag scales with
/// exit speed, and a supersonic exit would put the overshoot in the same
/// order as the drop test that calls apogee.
///
/// So the ring hands over a climb rate as well, and the seed is carried
/// forward to the exit sample with it. `peak_altitude_asl` is seeded from
/// the same number: it is a running maximum that only ever revises upward,
/// and seeding it where the airframe actually is beats seeding it half a
/// span behind.
///
/// RAM, measured: the `Deque` is 2072 B (128 * 16 B of `(u64, f32)` plus
/// its indices), and it sizes the whole `Stage` enum rather than just the
/// `MachLockout` variant, so `RocketStateEstimator` goes from 168 B to
/// 2232 B — +2064 B held for the entire flight, not only the lockout.
/// That is 0.2% of the STM32H743's RAM for the only mechanism standing
/// between one bad SPI read and a drogue at +109 m/s, and it is the whole
/// cost of the fix; nothing else was added.
///
/// The cap covers the span even at a 500 Hz feed (125 samples), which means
/// the span — wall-clock, like every other duration in this module — is
/// what bounds the window, not the cap, so a part running fast does not
/// quietly shorten it.
const BARO_RING_CAP: usize = 128;
const BARO_RING_SPAN_S: f32 = 0.25;

/// Per-rocket flight configuration for the deployment estimator.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct FlightProfile {
    /// Baro Mach lockout: starting at THIS estimator's ignition detection,
    /// the KF is dropped for this long and then rebuilt in flight from the
    /// first reading after it — supersonic static-port readings are garbage,
    /// and so is anything a filter carried into them.
    ///
    /// From the flight sim: time from ignition detection until decelerated
    /// back below Mach 0.75, with ~1.4x margin; it must still end well
    /// (>5 s) before apogee. `None` disables the lockout — use that for
    /// subsonic rockets.
    ///
    /// The Mach 0.75 here is deliberately lower than the airbrakes
    /// estimator's 0.8: a lower threshold is reached later, so this freeze
    /// runs longer than that estimator's lockout would. This half fires the
    /// pyros, so it buys its margin in time rather than in cleverness.
    ///
    /// Still not the same thing as
    /// [`MachLockoutConfig`](crate::airbrakes_estimator::MachLockoutConfig)
    /// despite the similar name — but no longer for the reason this comment
    /// used to give, which was that the two clocks ran from *different
    /// sensors*. They do not, and have not since `b901ace` deleted this
    /// half's barometric ignition detector. Both halves now run the one
    /// accelerometer detector in [`crate::ignition_detector`] — one
    /// implementation, two instances, the same 10 Hz low pass, the same
    /// 0.1 s sustain and — since 2026-08-18 — literally the same threshold:
    /// [`FlightConfig::ignition_detection_acc_threshold`](crate::FlightConfig::ignition_detection_acc_threshold)
    /// is one field above both halves, so no config can set them apart.
    /// With both halves free to act they latch on the same sample:
    /// `tests::osiris_sim::ignition_latch_time_by_threshold` sweeps 4-12 g
    /// and fails outright if the two ever disagree about which sample the
    /// motor lit on.
    ///
    /// **"Both free to act" is the caveat, and it is not hypothetical.**
    /// The airbrakes half refuses to detect ignition until its pad ring has
    /// filled AND its pad screening has produced a calibration; this half
    /// has neither precondition, because a board armed seconds before
    /// liftoff must still fire its pyros. So a short pad splits the two
    /// origins — on LC'25's 1.8 s of pad data
    /// (`airbrakes_estimator::tests::short_pad_refuses_ignition`) the
    /// airbrakes half never detects ignition at all, while this half is
    /// unaffected. The Osiris diagnostic can assert equality only because
    /// calibration there completes 54 s BEFORE ignition, so the gate never
    /// bites. Same sensor, same detector, same threshold — but not
    /// necessarily the same instant, and never this half waiting on that
    /// one.
    ///
    /// What stays independent by design is everything after the origin: the
    /// durations, the exit conditions, and the Mach numbers (0.75 here
    /// against the airbrakes' `max_open_mach`). This one is a plain
    /// duration — no filter exists until it expires and nothing measured in
    /// flight can shorten it — where the airbrakes pair is a window
    /// (earliest / forced) around a MEASURED drag check that decides
    /// somewhere inside it. Equal-looking numbers in a config are still
    /// coincidence rather than a link: they are answers to different
    /// questions that merely happen to be timed from the same event.
    pub mach_lockout_duration_us: Option<u32>,

    pub deployment: DeploymentProfile,
}

/// Deployment scheme: single (both pyros at apogee) or dual (drogue at
/// apogee, main at altitude).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub enum DeploymentProfile {
    /// Both pyros at apogee: after a single `delay_us` past descent detection, fire
    /// drogue then main back-to-back (main on the very next sample).
    Single {
        minimum_deployment_altitude_agl: f32,
        delay_us: u32,
    },
    /// Drogue at apogee, main at altitude.
    Dual {
        drogue_chute_minimum_altitude_agl: f32,
        drogue_chute_delay_us: u32,
        main_chute_altitude_agl: f32,
        main_chute_delay_us: u32,
    },
}

impl DeploymentProfile {
    fn minimum_deployment_agl(&self) -> f32 {
        match self {
            Self::Single {
                minimum_deployment_altitude_agl,
                ..
            } => *minimum_deployment_altitude_agl,
            Self::Dual {
                drogue_chute_minimum_altitude_agl,
                ..
            } => *drogue_chute_minimum_altitude_agl,
        }
    }

    fn drogue_delay_us(&self) -> u32 {
        match self {
            // Single: the one delay applies to the drogue (first) fire.
            Self::Single { delay_us, .. } => *delay_us,
            Self::Dual {
                drogue_chute_delay_us,
                ..
            } => *drogue_chute_delay_us,
        }
    }

    fn main_delay_us(&self) -> u32 {
        match self {
            // Single: main fires back-to-back with drogue (no extra delay).
            Self::Single { .. } => 0,
            Self::Dual {
                main_chute_delay_us,
                ..
            } => *main_chute_delay_us,
        }
    }

    fn is_single(&self) -> bool {
        matches!(self, Self::Single { .. })
    }

    fn main_chute_altitude_agl(&self) -> Option<f32> {
        match self {
            Self::Dual {
                main_chute_altitude_agl,
                ..
            } => Some(*main_chute_altitude_agl),
            Self::Single { .. } => None,
        }
    }
}

/// Vertical-only rocket state for telemetry / airbrakes.
///
/// Each variant carries only numbers that are live and trustworthy in that
/// state — most prominently, [`RocketState::MachLockout`] has no altitude
/// and no velocity, because the KF is frozen there and any value would be
/// stale. A caller that wants the filter's numbers without the state
/// machine wrapped around them — the SD log, mainly — reads
/// [`RocketStateEstimator::kf_altitude_asl`] /
/// [`RocketStateEstimator::kf_vertical_velocity`] instead, which go absent
/// over exactly the same window this variant set drops its fields in.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RocketState {
    OnPad,
    Ascent {
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    /// Ascending around/above Mach 1 with the baro locked out: the KF is
    /// frozen (no predict, no update), so there is no altitude or velocity
    /// to report — the fields are deliberately absent, making a stale read
    /// unrepresentable rather than merely discouraged. The pad altitude
    /// (latched at ignition detection) is the only live number.
    MachLockout { launch_pad_altitude_asl: f32 },
    DrogueChute {
        deployed: bool,
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    MainChute {
        deployed: bool,
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    Landed,
    FailedToReachMinApogee,
}

/// The launch pad altitude reference: the mean of one whole
/// [`PAD_WINDOW_S`] window of barometer, handed out one window late.
///
/// This is the only thing the barometer is for before the rocket moves,
/// and it is not a filter: there is nothing to estimate on a pad but a
/// constant, and a filter that exists only to smooth one is a filter
/// somebody can read a velocity out of.
///
/// **Handed out one window late** is the whole design. Windows close on
/// wall clock, and once two have closed the mean that
/// [`Self::reference_asl`] returns is the one before the one that closed
/// most recently — so it covers a full second that ended between one and
/// two seconds ago, and at the instant ignition latches it cannot contain
/// a single sample taken within a second of the motor lighting. Nothing an
/// igniter does to a static port is in it. That guard is why the previous
/// sample is not simply averaged up to ignition, and it costs nothing: the
/// rocket sits armed on the rail for minutes.
///
/// **Before two windows have closed** — the first ~2 s of a session, and
/// nowhere else — `reference_asl` degrades in order rather than going
/// absent: the one window that has closed, then the partial mean of the
/// window still filling. That keeps the promise every consumer downstream
/// is written against (`FlightEstimators::launch_pad_altitude_asl`, the
/// slow record's `launch_pad_altitude_asl`, the plot's row-wise AGL
/// conversion): a reference exists from the estimator's first sample
/// onward, and absence means "no estimator sample", never "too early". It
/// does mean the lag guarantee above holds only once those 2 s are up —
/// which is always true at ignition, because arming the board and walking
/// away takes minutes, not seconds.
///
/// **And no gate.** Until 2026-08-18 this was a 10 s low pass behind a
/// 100 m innovation gate with a 1 s resync, and the gate existed for one
/// thing — a bad SPI read decoding to pressure ~0, which is a ~30 km
/// reading. Nothing else can happen to a barometer on a rail: it is not
/// moving, baro noise is under a metre, and weather drift is metres over
/// minutes. The gate is gone because a gate on the pad has to solve the
/// problem it creates — anchor to a garbage first sample and it rejects
/// every honest reading after it, which is what the resync was for — and
/// that machinery is a worse thing to own than the fault it catches.
///
/// What that trades: one 30 km sample inside a window moves that window's
/// mean by 30000/416 = ~72 m, where the gated low pass held it to ~7 m.
/// Every AGL deployment decision is measured against this number. Nothing
/// in either archived flight log or either simulated Osiris motor contains
/// such a sample; if one ever appears, the answer is a median of the
/// window rather than a mean, which rejects nothing and needs no
/// threshold.
#[derive(Debug, Clone)]
struct PadReference {
    /// The reference itself: the mean of the second-most-recently closed
    /// window. `None` until two have closed, i.e. for the first ~2 s.
    reference_asl: Option<f32>,
    /// The most recently closed window's mean, waiting one window to
    /// become `reference_asl`.
    pending_asl: Option<f32>,
    /// Start of the window currently accumulating. `None` before the first
    /// sample.
    window_start_us: Option<u64>,
    /// Running sum of the current window, in `f64` because it is a sum: 416
    /// altitudes of ~1000 m accumulated in `f32` can round to metres, and
    /// this is the number every AGL deployment is measured against.
    sum: f64,
    count: u32,
}

impl PadReference {
    const fn new() -> Self {
        Self {
            reference_asl: None,
            pending_asl: None,
            window_start_us: None,
            sum: 0.0,
            count: 0,
        }
    }

    /// Feed one pad sample. Closes the current window first if this sample
    /// is past its end, so a window never contains a sample taken after it.
    fn push(&mut self, timestamp_us: u64, baro_altitude_asl: f32) {
        let start = *self.window_start_us.get_or_insert(timestamp_us);
        if (timestamp_us.saturating_sub(start)) as f32 * 1e-6 >= PAD_WINDOW_S && self.count > 0 {
            self.reference_asl = self.pending_asl;
            self.pending_asl = Some((self.sum / self.count as f64) as f32);
            self.window_start_us = Some(timestamp_us);
            self.sum = 0.0;
            self.count = 0;
        }
        self.sum += baro_altitude_asl as f64;
        self.count += 1;
    }

    /// The reference: the lagged window once there is one, else the best
    /// thing there is, else `None` before the very first sample. See the
    /// type doc for why this degrades rather than going absent.
    fn reference_asl(&self) -> Option<f32> {
        self.reference_asl
            .or(self.pending_asl)
            .or_else(|| (self.count > 0).then(|| (self.sum / self.count as f64) as f32))
    }
}

#[derive(Debug, Clone)]
enum Stage {
    OnPad { pad: PadReference },
    Ascent {
        launch_pad_altitude_asl: f32,
        /// running maximum of the filtered altitude; apogee is detected when
        /// the altitude drops [`APOGEE_DROP_M`] below it
        peak_altitude_asl: f32,
        /// how long the altitude has been continuously below
        /// (peak - APOGEE_DROP_M), in seconds of measured time
        below_peak_s: f32,
    },
    /// Baro readings are garbage around and above Mach 1 (shocks over the
    /// static port), and with baro as the only sensor there is no trustworthy
    /// signal to exit on — so this stage is entered directly at ignition
    /// detection (the COTS "Mach delay") and the KF is frozen (no predict, no
    /// update) for a sim-derived duration covering the whole fast regime,
    /// then re-seeded from fresh measurements. While frozen, no state
    /// transition can trigger.
    MachLockout {
        launch_pad_altitude_asl: f32,
        /// seconds of measured time still to wait
        remaining_s: f32,
        /// (timestamp_us, raw baro altitude) for the last
        /// [`BARO_RING_SPAN_S`] — the only thing kept from the lockout, and
        /// only so the filter can be born from a median instead of from
        /// whatever the one exit sample said. Nothing reads it before then.
        baro_ring: Deque<(u64, f32), BARO_RING_CAP>,
    },
    DrogueDelay {
        launch_pad_altitude_asl: f32,
        /// seconds of measured time still to wait
        remaining_s: f32,
    },
    DrogueDeployed {
        launch_pad_altitude_asl: f32,
    },
    MainDelay {
        launch_pad_altitude_asl: f32,
        /// seconds of measured time still to wait
        remaining_s: f32,
    },
    MainDeployed {
        launch_pad_altitude_asl: f32,
        /// how long |velocity| has been continuously below the landed
        /// threshold, in seconds of measured time
        still_s: f32,
    },
    Landed {
        launch_pad_altitude_asl: f32,
    },
    FailedToReachMinApogee {
        launch_pad_altitude_asl: f32,
    },
}

/// Deployment state estimator + flight state machine.
///
/// Feed it every timestamped baro altitude ASL sample via [`Self::update`].
/// The KF wants them at roughly [`SAMPLES_PER_S`] — it steps a fixed `DT`
/// per sample — but the state machine's timers read the timestamps, so the
/// actual rate does not have to be exactly that, and does not have to be
/// known.
#[derive(Debug, Clone)]
pub struct RocketStateEstimator {
    profile: FlightProfile,
    kf: Option<BaroAltitudeKF>,
    stage: Stage,
    /// Previous sample's timestamp, for the timer dt. `None` before the
    /// first sample.
    prev_timestamp_us: Option<u64>,
    /// This half's ignition detector. Its own instance, not shared with the
    /// airbrakes half's — see [`IgnitionDetector::update`].
    ignition: IgnitionDetector,
}

impl RocketStateEstimator {
    /// `ignition_detection_acc_threshold` is not part of [`FlightProfile`]
    /// because it is not this half's to own — it is
    /// [`FlightConfig::ignition_detection_acc_threshold`](crate::FlightConfig::ignition_detection_acc_threshold),
    /// the one number both halves detect ignition at, handed down by
    /// [`FlightEstimators::new`](crate::FlightEstimators::new).
    pub fn new(profile: FlightProfile, ignition_detection_acc_threshold: f32) -> Self {
        Self {
            profile,
            kf: None,
            stage: Stage::OnPad {
                pad: PadReference::new(),
            },
            prev_timestamp_us: None,
            ignition: IgnitionDetector::new(ignition_detection_acc_threshold),
        }
    }

    /// Measured time since the previous sample, for the state machine's
    /// timers only — the KF keeps its fixed `DT` step regardless.
    ///
    /// Deliberately unclamped. A long gap between samples is real elapsed
    /// time, and counting it is the entire point of reading a timestamp: a
    /// ceiling would put back exactly the error this removed, under-counting
    /// a Mach lockout or a pyro delay by whatever the sample stream lost.
    /// The one caller stamps every sample from the same monotonic clock, so
    /// a large delta means a gap, not a bad reading.
    ///
    /// Zero on the first sample, where there is no previous timestamp to
    /// difference against. Nothing is timing anything yet there, and the pad
    /// reference does not read this at all — its windows close on the
    /// timestamps themselves.
    fn timer_dt(&mut self, timestamp_us: u64) -> f32 {
        let dt = match self.prev_timestamp_us {
            Some(prev) => (timestamp_us.saturating_sub(prev)) as f32 * 1e-6,
            None => 0.0,
        };
        self.prev_timestamp_us = Some(timestamp_us);
        dt
    }

    /// Process one baro altitude ASL sample (m) with the timestamp it was
    /// taken at (us, same monotonic clock every call), and the raw
    /// accelerometer specific force from the same sample if there was one.
    ///
    /// `acc` is the RAW sensor vector, not anything another estimator
    /// derived from it, and it feeds exactly one decision: the ignition
    /// magnitude check (see
    /// [`FlightConfig::ignition_detection_acc_threshold`](crate::FlightConfig::ignition_detection_acc_threshold)).
    /// Nothing else in this estimator reads it, and it is the ONLY thing
    /// that can start a flight: `None` on every sample, or a dead IMU, and
    /// this estimator stays on the pad forever.
    ///
    /// Returns the pyro command for this sample — `Some(pyro)` when a pyro
    /// channel should be fired — and what the innovation gate did with this
    /// sample's baro reading. The gate outcome is returned rather than stored
    /// because a resync happens on exactly one sample and there is nowhere
    /// for it to go stale: see [`crate::BaroGateOutcome`]. `Accepted` covers
    /// every path where no gate ran at all — the Mach lockout, where nothing
    /// is fused, and the (unreachable) missing-filter fallback.
    ///
    /// The timestamp drives every *duration* below — the Mach lockout, the
    /// pyro delays, the apogee and landing persistence, the pad-altitude
    /// low pass — so all of them are in honest seconds no matter what the
    /// sensor's real output rate turns out to be, or how many samples the
    /// caller dropped getting here. The KF is untouched by it and still
    /// steps a fixed `DT` per sample; see [`SAMPLES_PER_S`] for why that
    /// split is deliberate rather than an oversight.
    pub fn update(
        &mut self,
        timestamp_us: u64,
        acc: Option<Vector3<f32>>,
        baro_altitude_asl: f32,
    ) -> (Option<PyroSelect>, BaroGateOutcome) {
        let dt = self.timer_dt(timestamp_us);
        // Run every sample so the low pass and the sustain are already warm
        // when the motor lights; the result is only consulted on the pad.
        let accel_says_ignition = self.ignition.update(acc, dt);

        // Mach lockout, handled here and not in the stage machine below,
        // because there is no filter during it to run that machine on.
        //
        // The filter is DROPPED at ignition detection rather than frozen.
        // Frozen was already the behaviour — predicting a constant-velocity
        // model through a 2-3 g deceleration would accumulate kilometres of
        // error, and the measurements it would fuse are shock garbage — but
        // a frozen filter is still an object holding pre-ignition numbers
        // that somebody can read. Dropping it makes "there is no altitude
        // here" a fact about the type rather than a rule to be remembered,
        // and it means the filter that comes out the other side has no
        // history to unlearn: it is built fresh, in flight, from the first
        // honest reading (`BaroAltitudeKF::born_in_flight`).
        //
        // Nothing else can happen in this window. No transition, no pyro.
        if let Stage::MachLockout {
            launch_pad_altitude_asl,
            remaining_s,
            baro_ring,
        } = &mut self.stage
        {
            // The reading goes in raw, shock garbage and all. There is
            // nothing to gate it against — the filter that owned the
            // innovation gate was dropped at ignition — and nothing reads
            // this ring except the median below, which does not care what
            // the samples it outvotes look like.
            if baro_ring.is_full() {
                baro_ring.pop_front();
            }
            let _ = baro_ring.push_back((timestamp_us, baro_altitude_asl));
            while let Some(front) = baro_ring.front() {
                if (timestamp_us.saturating_sub(front.0)) as f32 * 1e-6 > BARO_RING_SPAN_S {
                    baro_ring.pop_front();
                } else {
                    break;
                }
            }

            *remaining_s -= dt;
            if *remaining_s <= 0.0 {
                // Born from the last BARO_RING_SPAN_S rather than from this
                // one sample, so that one bad reading landing on the exit
                // sample cannot decide both where the filter starts and what
                // counts as the peak — see [`BARO_RING_CAP`] for the 28.67 s
                // drogue that costs. The ring gives up a climb rate as well
                // as an altitude, which is what lets the altitude be the one
                // at *this* sample rather than the one half a ring span ago;
                // see [`ring_birth_state`].
                //
                // `unwrap_or` cannot fire: this sample was just pushed. It is
                // spelt as a fallback rather than an `expect` because this is
                // the code path that fires pyros, and the honest degradation
                // of an empty ring is the old single-sample behaviour, not a
                // panic that resets the board mid-flight.
                let (seed_asl, seed_velocity) =
                    ring_birth_state(baro_ring, timestamp_us).unwrap_or((baro_altitude_asl, 0.0));
                log_info!(
                    "mach lockout over, building KF in flight at {}m climbing {}m/s (from the last {}s; this sample read {}m)",
                    seed_asl,
                    seed_velocity,
                    BARO_RING_SPAN_S,
                    baro_altitude_asl
                );
                self.kf = Some(BaroAltitudeKF::born_in_flight(seed_asl, seed_velocity));
                self.stage = Stage::Ascent {
                    launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    peak_altitude_asl: seed_asl,
                    below_peak_s: 0.0,
                };
            }
            // Nothing is fused during the lockout, so nothing is rejected.
            return (None, BaroGateOutcome::Accepted);
        }

        // On the pad, also handled before the stage machine, and for the
        // same reason: no filter exists yet. The barometer's only job here
        // is the pad altitude reference, tracked directly.
        if let Stage::OnPad { pad } = &mut self.stage {
            // Every sample goes in, ungated — see `PadReference`. Nothing
            // is fused on the pad, so nothing can be rejected here.
            pad.push(timestamp_us, baro_altitude_asl);
            // Copied out so the borrow of `self.stage` ends here. The
            // fallback is for a launch inside the first two windows, which
            // means an armed board that has been powered for under two
            // seconds; the honest degradation is this one reading, not a
            // number from a window that does not exist.
            let pad = pad.reference_asl().unwrap_or(baro_altitude_asl);

            if accel_says_ignition {
                log_info!("ignition detected by accel, pad asl={}m", pad);
                self.stage = match self.profile.mach_lockout_duration_us {
                    Some(duration_us) => {
                        log_info!("mach lockout for {}us, no KF until it ends", duration_us);
                        Stage::MachLockout {
                            launch_pad_altitude_asl: pad,
                            remaining_s: duration_us as f32 * 1e-6,
                            baro_ring: Deque::new(),
                        }
                    }
                    // Subsonic profile: the static port never stops telling
                    // the truth, so the filter starts now — at rest, which
                    // is what it is, the rocket having moved centimetres in
                    // the ~0.15 s the detector took.
                    //
                    // Seeded from `pad`, not from this sample's raw reading,
                    // for the reason the lockout exit above is seeded from a
                    // median: a birth reads one number and then defends it
                    // with an innovation gate, so that number must not be
                    // able to be a single bad SPI read. `pad` is the mean of
                    // a whole second of them, and on a rocket that has moved
                    // centimetres it is the better estimate of where the
                    // rocket is anyway.
                    None => {
                        self.kf = Some(BaroAltitudeKF::new(pad));
                        Stage::Ascent {
                            launch_pad_altitude_asl: pad,
                            peak_altitude_asl: pad,
                            below_peak_s: 0.0,
                        }
                    }
                };
            }
            return (None, BaroGateOutcome::Accepted);
        }

        // Past the pad and past the lockout, a filter always exists. If it
        // somehow does not, do nothing rather than panic: this is the code
        // path that fires pyros. Nothing was fused, so nothing was rejected.
        let Some(kf) = self.kf.as_mut() else {
            return (None, BaroGateOutcome::Accepted);
        };
        kf.predict();
        let gate = kf.update(baro_altitude_asl);
        let altitude_asl = kf.altitude_asl();
        let velocity = kf.vertical_velocity();

        let mut deploy_pyro = None;

        match &mut self.stage {
            // Handled before the filter runs, above: there is none on the pad.
            Stage::OnPad { .. } => {}
            Stage::Ascent {
                launch_pad_altitude_asl,
                peak_altitude_asl,
                below_peak_s,
            } => {
                if altitude_asl > *peak_altitude_asl {
                    *peak_altitude_asl = altitude_asl;
                }

                if *peak_altitude_asl - altitude_asl > APOGEE_DROP_M {
                    *below_peak_s += dt;
                } else {
                    *below_peak_s = 0.0;
                }

                if *below_peak_s >= APOGEE_DROP_SUSTAIN_S {
                    let apogee_agl = *peak_altitude_asl - *launch_pad_altitude_asl;
                    let min_agl = self.profile.deployment.minimum_deployment_agl();
                    if apogee_agl < min_agl {
                        log_info!(
                            "failed to reach min apogee: min={}, peak={}",
                            min_agl,
                            apogee_agl
                        );
                        self.stage = Stage::FailedToReachMinApogee {
                            launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        };
                    } else {
                        log_info!("descent detected: peak agl={}m", apogee_agl);
                        self.stage = Stage::DrogueDelay {
                            launch_pad_altitude_asl: *launch_pad_altitude_asl,
                            remaining_s: self.profile.deployment.drogue_delay_us() as f32 * 1e-6,
                        };
                    }
                }
            }
            // Unreachable: handled at the top of `update`, which returns
            // before this match, because there is no filter to run the stage
            // machine on during the lockout. Deliberately not `unreachable!`
            // — this is the code path that fires pyros, and a panic here
            // would reset the board mid-flight over a logic error whose
            // honest degradation is simply staying locked out.
            Stage::MachLockout { .. } => {}
            Stage::DrogueDelay {
                launch_pad_altitude_asl,
                remaining_s,
            } => {
                if *remaining_s <= 0.0 {
                    deploy_pyro = Some(PyroSelect::PyroDrogue);
                    let pad = *launch_pad_altitude_asl;
                    if self.profile.deployment.is_single() {
                        // Single: main follows drogue with no extra delay
                        // (main_delay_us() == 0), so it fires on the next sample.
                        self.stage = Stage::MainDelay {
                            launch_pad_altitude_asl: pad,
                            remaining_s: self.profile.deployment.main_delay_us() as f32 * 1e-6,
                        };
                    } else {
                        self.stage = Stage::DrogueDeployed {
                            launch_pad_altitude_asl: pad,
                        };
                    }
                } else {
                    *remaining_s -= dt;
                }
            }
            Stage::DrogueDeployed {
                launch_pad_altitude_asl,
            } => {
                // Dual only: wait for main altitude.
                if let Some(main_agl) = self.profile.deployment.main_chute_altitude_agl()
                    && altitude_asl < main_agl + *launch_pad_altitude_asl
                {
                    self.stage = Stage::MainDelay {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        remaining_s: self.profile.deployment.main_delay_us() as f32 * 1e-6,
                    };
                }
            }
            Stage::MainDelay {
                launch_pad_altitude_asl,
                remaining_s,
            } => {
                if *remaining_s <= 0.0 {
                    deploy_pyro = Some(PyroSelect::PyroMain);
                    self.stage = Stage::MainDeployed {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        still_s: 0.0,
                    };
                } else {
                    *remaining_s -= dt;
                }
            }
            Stage::MainDeployed {
                launch_pad_altitude_asl,
                still_s,
            } => {
                if velocity.abs() < LANDED_VELOCITY_THRESHOLD {
                    *still_s += dt;
                } else {
                    *still_s = 0.0;
                }

                if *still_s >= LANDED_DETECTION_S {
                    log_info!("landed");
                    self.stage = Stage::Landed {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Stage::Landed { .. } | Stage::FailedToReachMinApogee { .. } => {}
        }

        (deploy_pyro, gate)
    }

    pub fn state(&self) -> RocketState {
        let (altitude_asl, velocity) = match &self.kf {
            Some(kf) => (kf.altitude_asl(), kf.vertical_velocity()),
            None => (0.0, 0.0),
        };

        match &self.stage {
            Stage::OnPad { .. } => RocketState::OnPad,
            Stage::Ascent {
                launch_pad_altitude_asl,
                ..
            } => RocketState::Ascent {
                vertical_velocity: velocity,
                altitude_asl,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            // The frozen KF values are deliberately NOT reported here — the
            // variant has no fields to put them in (see [`RocketState`]).
            Stage::MachLockout {
                launch_pad_altitude_asl,
                ..
            } => RocketState::MachLockout {
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::DrogueDelay {
                launch_pad_altitude_asl,
                ..
            } => RocketState::DrogueChute {
                deployed: false,
                vertical_velocity: velocity,
                altitude_asl,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::DrogueDeployed {
                launch_pad_altitude_asl,
            } => RocketState::DrogueChute {
                deployed: true,
                vertical_velocity: velocity,
                altitude_asl,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::MainDelay {
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: false,
                vertical_velocity: velocity,
                altitude_asl,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::MainDeployed {
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: true,
                vertical_velocity: velocity,
                altitude_asl,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::Landed { .. } => RocketState::Landed,
            Stage::FailedToReachMinApogee { .. } => RocketState::FailedToReachMinApogee,
        }
    }

    /// KF altitude ASL (m) for the fast flight-log record, or `None` when
    /// there is genuinely no altitude to report — which is most of the
    /// prelaunch and boost record, not a corner case.
    ///
    /// `self.kf` is `None` for the WHOLE pad period, however long the rocket
    /// sits armed: nothing builds a filter until ignition is detected, and
    /// the only thing the barometer estimates before then is
    /// [`Self::launch_pad_altitude_asl`]. On a Mach profile it is then
    /// `None` again for the entire lockout — the filter is dropped at
    /// ignition rather than frozen, so on `FLIGHT_CONFIG` that is a further
    /// 26 s with no altitude at all — and a filter first exists at the
    /// instant the lockout ends. (On a subsonic profile, `mach_lockout` =
    /// `None`, the filter is born at ignition detection and there is no gap.)
    ///
    /// This doc used to claim the opposite — that every stage but the
    /// lockout returned `Some`, `OnPad` included, because "the filter is
    /// running and fusing baro in all of them". It never was on the pad; the
    /// pad reference is a windowed mean, not a filter, and
    /// `kf_accessors_absent_before_birth_and_during_lockout` asserts the
    /// `None`. Written down because a log reader that expects an altitude
    /// here would read the whole pad segment as missing data rather than as
    /// the answer.
    ///
    /// From the lockout's end onward every stage returns `Some`, including
    /// `Landed` and `FailedToReachMinApogee`, whose [`RocketState`] variants
    /// carry no altitude field of their own: the filter is live there and the
    /// number means what it says; those variants omit it because the state
    /// machine has nothing to *decide* from it, not because it is stale.
    ///
    /// Still prefer [`Self::state`] for anything that acts on the value —
    /// not because this accessor is less honest, but because `state()` hands
    /// over the stage and the numbers as one value, so a caller cannot read
    /// an altitude without also learning which flight phase produced it.
    pub fn kf_altitude_asl(&self) -> Option<f32> {
        self.kf.as_ref().map(|kf| kf.altitude_asl())
    }

    /// KF vertical velocity (m/s) for the fast flight-log record. Present in
    /// exactly the samples [`Self::kf_altitude_asl`] is present in, and
    /// absent for the same two reasons — no filter yet, or no filter during
    /// the Mach lockout — so the logged altitude and velocity always come
    /// from one filter or from none, and never from a half of each.
    /// [`Self::state`] remains the interface to prefer for control, for the
    /// stage-plus-numbers reason given there.
    pub fn kf_vertical_velocity(&self) -> Option<f32> {
        self.kf.as_ref().map(|kf| kf.vertical_velocity())
    }

    /// Launch pad altitude ASL (m), available in every stage: while on the
    /// pad this is [`PadReference`]'s current mean — one second of
    /// barometer, stepping once a second, from a window that ended a second
    /// ago; from ignition detection onward it is the value latched at
    /// detection.
    pub fn launch_pad_altitude_asl(&self) -> f32 {
        match &self.stage {
            // Zero before the first sample and nowhere else — see
            // `PadReference::reference_asl`, which degrades through the
            // partial window rather than going absent for the first ~2 s.
            Stage::OnPad { pad } => pad.reference_asl().unwrap_or(0.0),
            Stage::Ascent {
                launch_pad_altitude_asl,
                ..
            }
            | Stage::MachLockout {
                launch_pad_altitude_asl,
                ..
            }
            | Stage::DrogueDelay {
                launch_pad_altitude_asl,
                ..
            }
            | Stage::DrogueDeployed {
                launch_pad_altitude_asl,
            }
            | Stage::MainDelay {
                launch_pad_altitude_asl,
                ..
            }
            | Stage::MainDeployed {
                launch_pad_altitude_asl,
                ..
            }
            | Stage::Landed {
                launch_pad_altitude_asl,
            }
            | Stage::FailedToReachMinApogee {
                launch_pad_altitude_asl,
            } => *launch_pad_altitude_asl,
        }
    }
}

/// Median of 9 samples spaced evenly across the ring — a transient cannot
/// move it, unlike a mean or a single reading. `None` on an empty ring.
///
/// Nine picks rather than the whole ring because the ring holds ~104
/// samples at 416 Hz and this runs on the sample that ends the Mach
/// lockout: nine `f32`s sort on the stack in constant time, and the
/// robustness a median buys is set by how many of the picks a transient can
/// reach, not by how many samples were available to pick from.
///
/// The airbrakes half's `ring_median` solves the lag by carrying every pick
/// forward with the dead reckoner's velocity. There is no reckoner on this
/// side — the deployment half is baro-only and may not call into a half that
/// can be retired mid-flight — so the ring supplies its own rate: median the
/// older half of the picks, median the newer half, and the difference over
/// the gap between them is the climb. Medians on both sides, so the property
/// the single median was chosen for survives intact — a transient still has
/// to outvote a half to be believed.
///
fn ring_birth_state(
    ring: &Deque<(u64, f32), BARO_RING_CAP>,
    now_us: u64,
) -> Option<(f32, f32)> {
    let n = ring.len();
    if n == 0 {
        return None;
    }
    let mut picks = [(0u64, 0.0f32); 9];
    let mut count = 0usize;
    for (i, sample) in ring.iter().enumerate() {
        while count < 9 && i == (count * (n - 1)) / 8 {
            picks[count] = *sample;
            count += 1;
        }
    }
    let picks = &mut picks[..count];
    // The middle pick belongs to neither half. The rate is two medians
    // differenced over the gap between them, so the gap is worth keeping as
    // wide as the ring allows — it is what divides their noise.
    let half = (count / 2).max(1);
    let older = median_by_altitude(&mut picks[..half]);
    let newer = median_by_altitude(&mut picks[count - half..]);

    let gap_s = (newer.0.saturating_sub(older.0)) as f32 * 1e-6;
    // A one-sample ring, or picks that all carry one timestamp: no rate can
    // be taken, and the honest answer is the one this function used to give
    // — the median, at rest, lag and all.
    let climb = if gap_s > 0.0 {
        (newer.1 - older.1) / gap_s
    } else {
        0.0
    };
    let age_s = (now_us.saturating_sub(newer.0)) as f32 * 1e-6;
    Some((newer.1 + climb * age_s, climb))
}

/// The middle sample of a few, ranked by altitude, with its own timestamp.
///
/// The timestamp is the point: it is what the two halves are differenced
/// over. Ranking by altitude and reading off the time is sound because the
/// airframe is climbing monotonically through a 0.25 s window at a few
/// hundred m/s — the picks are tens of metres apart against 0.36 m of baro
/// noise, so their altitude order *is* their time order, and a pick that
/// somehow ranks out of order is one the median did not choose.
fn median_by_altitude(picks: &mut [(u64, f32)]) -> (u64, f32) {
    // `total_cmp`, not `partial_cmp(..).unwrap()`: this half cannot be
    // retired mid-flight, so a panic here is the flight. A NaN altitude
    // should not reach the ring at all — VLF5's `sensor_tasks.rs` rejects a
    // non-finite or non-positive pressure as a failed read — but that guard
    // lives in another repo, and the cost of not depending on it is one
    // token. `total_cmp` orders NaN rather than refusing to; the median of a
    // ring that somehow contains one is then merely wrong, not fatal.
    picks.sort_unstable_by(|a, b| a.1.total_cmp(&b.1));
    picks[picks.len() / 2]
}
