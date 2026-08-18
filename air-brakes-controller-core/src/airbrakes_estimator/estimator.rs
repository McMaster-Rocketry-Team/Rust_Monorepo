use heapless::Deque;
use nalgebra::{SVector, Vector2, Vector3};

use firmware_common_new::flight_data_record::AirbrakesState;

use crate::{
    airbrakes_estimator::{
        AirbrakesConfig, ImuSample, MAX_DT_S, dead_reckoner::DeadReckoner,
        vertical_kf::VerticalKF,
    },
    baro_gate::BaroGateOutcome,
    ignition_detector::IgnitionDetector,
    utils::{approximate_air_density, approximate_speed_of_sound},
};

// --- Pad calibration (Piece 1) ---------------------------------------------
// Everything flight needs to learn while the rocket is still — gyro bias
// and gravity direction (pad orientation) — comes from ONE structure:
// finished 2 s window means (gyro + accel together), averaged over the
// windows that look like a rocket sitting on a rail. The pad ALTITUDE is
// not among them; see the note on `PadCalibration` for where it lives.
// There is no fallback path: until enough windows qualify, the estimator
// is not launch-ready and refuses to detect ignition (see
// `calibration_complete`).
//
// The screen is ABSOLUTE — "is this a pad window at all" — and not a
// comparison between windows. It has to be, because the case it exists
// for is a recorder that started too late to see the pad: ignition is
// never detected, the estimator stays here, and it goes on closing
// windows THROUGH THE FLIGHT (`short_pad_refuses_ignition`, on the raw
// LC'25 log, whose pad segment is 1.2 s). Screening those out by mutual
// disagreement worked only because they happened to disagree with each
// other; a log that was uniformly in flight would have calibrated.
//
// This used to be a median across the windows plus a per-channel reject
// radius. Measured on Void Lake — the only log in the archive with a real
// pad segment (11.4 s, five windows) — that apparatus moved the gyro bias
// 0.03 deg/s and the gravity direction 0.002 deg, against a 5 deg tilt
// budget, and moved the (since removed) pad altitude 0.1 m the WRONG way.
// Deleting it
// outright left 45 of 46 tests passing; the 46th is the in-flight-windows
// case above, which the absolute check covers directly. The one
// disturbance it ever fired on was Void Lake's ~8 Hz airframe ring, 19
// deg/s peak at 0.3 deg of amplitude — which a 2 s mean already averages
// away, and which never moved the accelerometer more than 0.1 m/s^2.
/// Each completed calibration window is this long (s of measured time).
const PAD_WINDOW_S: f32 = 2.0;
/// How many window means we keep (32 x 2 s = about the last minute on pad).
const MAX_PAD_WINDOWS: usize = 32;
/// A pad window's mean specific force must be this close to 1 g.
///
/// Sized by accelerometer SCALE error, not by motion: the pads in the
/// archive read |a| = 10.00 m/s^2 (Void Lake) and 9.75 (LC'25), so the
/// radius has to swallow ~0.25 m/s^2 of calibration error before it can
/// reject anything at all. What it catches is a window that is not a pad:
/// coasting (|a| ~ 0), boosting, or descending under drogue.
const PAD_GRAVITY_TOLERANCE_M_S2: f32 = 0.5;
/// ...and a pad window's mean rotation rate must be under this.
///
/// Generous for the same kind of reason: this is a MEAN, so an
/// uncalibrated gyro bias lands in it whole (Void Lake's pad reads
/// 2.05 deg/s of pure bias). Sway does not — the mean of a rate over a
/// window is net rotation divided by the window, so an oscillation
/// averages toward zero. The 2 s window, not this threshold, is what
/// rejects motion. In-flight windows are tens to hundreds of deg/s and
/// are nowhere near a near miss.
const PAD_ROTATION_LIMIT_RAD_S: f32 = 0.174_53; // 10 deg/s
/// The pre-ignition rolling buffer spans this much MEASURED time. On
/// ignition the whole span is replayed through the dead reckoner (the
/// "rewind"), which is what recovers the thrust that had already built up
/// while the low pass and the threshold were still catching up.
///
/// This is a SIZED number, not a generous one, and it was 2.0 s until
/// 2026-08-17. The rewind trades two errors against each other: replay too
/// little and the thrust before detection is lost outright, replay too much
/// and every extra second of still rail is integrated against a hard-coded
/// 9.81 that the pad does not actually read. Both archived logs measure
/// their own scale error — Void Lake's pad reads |g| = 9.9967 and LC'25's
/// 9.7529 — so a stationary second injects +0.187 and -0.057 m/s.
///
/// Measured error in the recovered delta-vz against thrust-only truth, i.e.
/// the same integral with each log's own pad |g| removed instead of 9.81:
///
/// | ring span | Void Lake | LC'25   |
/// |-----------|-----------|---------|
/// | 0 s       | -15.129   | -6.973  |
/// | 0.10 s    |  -1.026   | -0.972  |
/// | **0.25 s**| **+0.016**|**-0.016**|
/// | 0.50 s    |  +0.073   | -0.028  |
/// | 1.0 s     |  +0.168   | -0.057  |
/// | 2.0 s     |  +0.345   | -0.114  |
///
/// 0.25 s is where the two errors cross: it is long enough to cover the
/// detector's own lag (~0.15 s of low pass plus the 0.1 s shared sustain),
/// and short enough that the scale error it replays is nothing. Deleting
/// the ring is NOT the alternative — that loses the whole pre-detection
/// delta-vz, 15.1 m/s on Void Lake against a 15 m/s birth sigma, which is
/// three orders worse than sizing it correctly.
const PAD_RING_SPAN_S: f32 = 0.25;
/// Capacity for that span. Trimming is by time, not by count, so this only
/// has to be large enough that a faster-than-nominal sensor still buffers
/// the whole span instead of quietly rewinding less of it: 128 covers
/// 0.25 s at up to 512 Hz. Past that the ring saturates and the rewind gets
/// shorter, which is exactly what the old count-based buffer did at ANY
/// rate above nominal — degraded, not broken.
const PAD_RING_CAP: usize = 128;
// The ignition low pass, threshold and sustain live in
// [`crate::ignition_detector`], shared with the pyro half — this file owns
// only the calibration gate in front of the result (`State::Armed`) and the
// rewind behind it.
//
// What stays true here is that detection is the ORIGIN every lockout timer
// is measured from ([`MachLockoutConfig`]), while the constants themselves
// come from the simulation's time-since-ignition. Any lag in the detector
// biases all of them late by exactly that lag, so the detector's timing is
// this file's business even though its code is not. The shared sustain adds
// 0.1 s of deliberate lag on top of the low pass's own; that is spent out of
// the drag check's margin, which `clipped_accel_still_flies_the_profile`
// bounds.
//
// [`MachLockoutConfig`]: crate::airbrakes_estimator::MachLockoutConfig

/// The calibration exists once this many windows pass the screen. Three
/// windows are 6 s of pad data, which is what buys the averaging — the
/// screen itself is a plausibility test and does not sharpen the numbers.
/// The rocket sits armed on the rail for minutes, so waiting for window 3
/// costs nothing operationally.
const MIN_CALIBRATION_WINDOWS: usize = 3;

// --- Stage 1 (thrust-vector alignment) ------------------------------------
const STAGE1_DURATION_S: f32 = 0.5;

// --- Lockout exit: the drag check (Piece 3) --------------------------------
// The Mach the check votes at is per-airframe and lives in
// `AirbrakesConfig::max_open_mach` — the same value the MPC gate uses,
// because "the flow is subsonic" and "the flaps may open" are one fact
// about the airframe. It is the Mach the config's own Cd table is
// tabulated at, so the threshold and the Cd travel together.
/// The drag check must hold continuously this long before the baro is
/// declared honest.
const SUBSONIC_SUSTAIN_S: f32 = 1.0;
/// Burnout is latched when the axial channel — deceleration-positive, the
/// same `a_axial` the drag check inverts — has been at least this positive
/// continuously for `BURNOUT_SUSTAIN_S`.
///
/// The SIGN is the discriminator, not the magnitude: thrust acts along
/// +axis and drag along -axis, so the channel crosses zero at burnout and
/// stays decelerating for the whole coast. A magnitude test cannot tell
/// 11.7 m/s^2 of thrust-minus-drag from 11.7 m/s^2 of pure drag — measured
/// on LC'25 at ignition+6.00 s, where inverting |accel| yields a confident
/// and completely wrong Mach 0.91 while this channel reads -10.66 and
/// correctly says "still burning". That measurement is why the drag check
/// reads this channel too, rather than the magnitude it used to.
///
/// 2 m/s^2 rather than 0 keeps the latch clear of the crossing itself.
/// Coast drag is 7-21 m/s^2 over the region that matters, and the pad noise
/// floor is ~0.04 m/s^2, so there is no contest.
const BURNOUT_DECEL_M_S2: f32 = 2.0;
/// How long the axial channel must stay decelerating before burnout latches.
/// Measured latch times: LC'25 ignition+6.38 s, Void Lake +1.96 s — both
/// about this long after true burnout, i.e. erring late.
const BURNOUT_SUSTAIN_S: f32 = 0.3;
/// Time constant of the low pass on the drag channel. The channel is a
/// single raw sample, so it carries the full accelerometer noise and
/// airframe vibration; unfiltered, one noisy sample trips the threshold
/// about a second early (measured on LC'25: 10.1 s vs 11.6 s).
const DRAG_LP_TAU_S: f32 = 0.3;
/// Ring of recent raw baro samples, used only to pick the birth altitude
/// by median. Covers `BARO_RING_SPAN_S` even at a 500 Hz feed.
const BARO_RING_CAP: usize = 512;
pub(super) const BARO_RING_SPAN_S: f32 = 0.7;

// --- Birth of the vertical filter (Piece 4) -------------------------------
/// Initial vertical-velocity uncertainty at a check-approved birth (the
/// dead-reckoned velocity is trusted to roughly this, m/s std).
const BORN_VELOCITY_STD: f32 = 15.0;
/// ...and at a forced (T_max) birth, where the dead reckoner may have
/// drifted badly: bigger, so the baro pulls velocity back fast.
const FORCED_BORN_VELOCITY_STD: f32 = 30.0;
const KF_Q_ACCEL_STD: f32 = 0.5;
const KF_R_ALT_STD: f32 = 3.0;

// --- Apogee ----------------------------------------------------------------
// There is no apogee latch here any more, and the constants that ran it
// (vertical velocity below 1 m/s sustained 0.5 s, guarded by a 0.25 s baro
// freshness and a 0.5 s frozen-baro watch) are gone with it. It was dead in
// flight, twice over:
//
// * `FlightEstimators::update` retires this half at `velocity().y <= 0`, and
//   that fired FIRST on both real logs — by 0.389 s on Void Lake and 0.392 s
//   on LC'25. The latch never got to run in a composed flight.
// * It could not have fired anyway. The sustain needs 0.5 s below 1 m/s and
//   the trajectory only offers 0.108 s (Void Lake) / 0.106 s (LC'25) between
//   crossing 1 m/s and crossing zero — a rocket decelerating at ~9.8 m/s^2
//   spends about a tenth of a second in that band, so 0.5 s was
//   unsatisfiable, not merely slow.
//
// And had it somehow won the race it would have been actively harmful:
// `velocity()` returned `None` in the apogee state (measured: `Some` on 0
// samples after the latch on both logs), so `descending` in
// `FlightEstimators::update` would have gone permanently false and the ONLY
// remaining retirement conditions would have been tilt past the horizon and
// the deployment half's apogee call.

// --- Outputs ---------------------------------------------------------------
/// Tilt is capped here for the horizontal-velocity output (tan blows up at
/// 90 deg, where the brakes have no authority left anyway).
const TILT_CAP_RAD: f32 = 1.396; // 80 deg

/// The screened pad calibration: everything flight needs to know that can
/// only be learned while the rocket is still. Produced (and continually
/// refreshed) by `screen_pad_windows` as pad windows finish.
#[derive(Debug, Clone, Copy)]
struct PadCalibration {
    /// Mean gyro of the surviving windows (rad/s).
    gyro_bias: Vector3<f32>,
    /// Mean accel of the surviving windows: gravity in the avionics
    /// frame, i.e. the pad orientation.
    gravity_av_frame: Vector3<f32>,
}

// The pad ALTITUDE is deliberately not in there, and this half does not
// measure one. There is exactly one pad altitude in the system — the
// deployment half's, reached through
// [`FlightEstimators::launch_pad_altitude_asl`] — and it is what every AGL
// number in the firmware is measured from: the pyro thresholds, the
// downlink, the MPC's apogee target and the SD log.
//
// This half used to average its own out of the same screened windows and
// carry it through every state after ignition. Nothing ever read it — the
// vertical filter is born at the baro ring's median, not at the pad — and
// the two numbers are derived differently enough to disagree: a 10 s low
// pass over the whole pad period against the mean of the last ~minute of
// 2 s windows. A future caller reaching for whichever accessor was nearer
// would have silently picked up a second, differently-wrong reference. One
// number, one owner.
//
// [`FlightEstimators::launch_pad_altitude_asl`]: crate::FlightEstimators::launch_pad_altitude_asl

/// One buffered pre-ignition sample: everything the rewind replays, and
/// nothing else. The baro altitude is deliberately absent — the rewind
/// only drives the dead reckoner, which carries no altitude. At 32 bytes
/// against the 40 a timestamp + [`ImuSample`] + altitude would take,
/// dropping it pays for the ring's capacity headroom outright.
#[derive(Debug, Clone, Copy)]
struct PadSample {
    timestamp_us: u64,
    acc: Vector3<f32>,
    gyro: Vector3<f32>,
}

/// Four states, in the order the flight passes through them, and it only ever
/// passes forward: `Armed` -> `Stage1` -> `DeadReckoning` ->
/// `AirbrakesEnabled`. There is no path back from any of them and no fifth —
/// the estimator's life ends by being dropped whole at apogee (see
/// [`FlightEstimators::update`]), not by transitioning.
///
/// That is a stronger claim than "the transitions happen to be written
/// one-way". Everything that could have gone backwards is a transition
/// condition rather than a live one: the brakes' permission used to be
/// recomputed every sample downstream and could withdraw itself, which meant
/// a filter transient could shut the brakes after they had opened. The Mach
/// limit that permission turned on is now checked once, on the way into
/// `AirbrakesEnabled`, and the state is what carries the answer afterwards.
///
/// [`FlightEstimators::update`]: crate::FlightEstimators::update
#[derive(Debug)]
enum State {
    /// Armed on the pad: collect screened calibration windows (gyro bias,
    /// gravity direction), and watch for ignition with a
    /// 0.25 s rolling buffer that is rewound once ignition is detected. The
    /// rolling buffer's ONLY job is that rewind — calibration comes
    /// entirely from the windows.
    Armed {
        pad_ring: Deque<PadSample, PAD_RING_CAP>,
        /// This half's ignition detector. Its own instance, not shared with
        /// the pyro half's — see [`IgnitionDetector::update`].
        ignition: IgnitionDetector,
        /// Finished window means, packed acc[0..3], gyro[3..6] — one
        /// vector so both channels are averaged and screened over exactly
        /// the same window.
        pad_windows: heapless::Vec<SVector<f32, 6>, MAX_PAD_WINDOWS>,
        /// Running sum and sample count of the window in progress; its mean
        /// is `window_sum / window_n`.
        ///
        /// This was Welford's incremental mean until 2026-08-17, on the
        /// stated grounds that a plain sum of thousands of ~9.81 m/s^2
        /// samples loses the hundredths the screen reads. Measured on the
        /// real windows of both archived logs, the plain sum's error in
        /// `|acc|` is 2.7e-6 (Void Lake, 385 samples) and 1.06e-5 (LC'25,
        /// 1000 samples) — four orders below the hundredths, and within a
        /// factor of two of Welford's own 1.5e-6 / 7.6e-6. Past ~10^5
        /// samples, a window this code cannot produce, the two trade places
        /// in both directions: Welford's `1/n` increment underflows just as
        /// the sum loses digits.
        window_sum: SVector<f32, 6>,
        window_n: u32,
        window_elapsed: f32,
        /// Latest screening result; `None` until enough windows agree.
        /// Ignition detection is refused while this is `None`.
        calibration: Option<PadCalibration>,
    },

    /// First half second of powered flight: the thrust direction tells us
    /// how the avionics are mounted in the rocket.
    Stage1 {
        elapsed: f32,
        /// Running SUM of the stage-1 specific force, never divided by a
        /// count. Both things the stage-1 mean feeds are scale-invariant —
        /// an angle and a `normalize()` — so the division would only round
        /// the answer. See the exit below for the one consumer that IS
        /// magnitude-sensitive and why normalizing there is what makes it
        /// safe.
        acc_sum: Vector3<f32>,
        /// Earth UP in the avionics frame as the PAD measured it. Kept only
        /// so the launch angle is logged against the rail's attitude rather
        /// than against the reckoner's half-second-old one.
        pad_up_av: Vector3<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        ignition_t_us: u64,
    },

    /// Boost and Mach lockout: inertial dead reckoning only, no filter.
    /// The baro is buffered (to pick a birth altitude) but never fused.
    DeadReckoning {
        /// Unit airframe axis in the avionics frame, from the stage-1 mean
        /// thrust direction. Thrust is +, drag is -. It is also the whole
        /// of the mounting solution the tilt output needs: tilt is the
        /// angle between this and the dead reckoner's `up_av`.
        thrust_axis_av: Vector3<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        ignition_t_us: u64,
        /// (timestamp_us, raw baro altitude)
        baro_ring: Deque<(u64, f32), BARO_RING_CAP>,
        subsonic_sustain: f32,
        /// Low-passed axial specific force, deceleration-positive —
        /// drag/mass once the motor is out, and negative while it is not.
        /// The same channel `burnout` latches on, read through the low pass
        /// rather than raw. `None` until the first sample of this state.
        drag_lp: Option<f32>,
        last_subsonic: bool,
        /// Latched once the axial channel proves the motor is out. Nothing
        /// downstream — neither the drag check nor the subsonic birth — is
        /// allowed to proceed before it.
        burnout: bool,
        burnout_sustain: f32,
    },

    /// The brakes may open, and will be allowed to for the rest of the
    /// flight: the baro is honest, the 2-state vertical filter exists and
    /// runs to apogee, and the Mach limit was cleared on the way in. Tilt
    /// still comes from the gyro dead reckoner.
    ///
    /// This is the LAST state. There is no apogee state to move on to: the
    /// whole estimator is dropped by [`FlightEstimators::update`] the first
    /// sample the filter's vertical velocity reaches zero, so the ascent is
    /// all this state ever sees.
    ///
    /// [`FlightEstimators::update`]: crate::FlightEstimators::update
    AirbrakesEnabled {
        thrust_axis_av: Vector3<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        kf: VerticalKF,
        born_t_us: u64,
        born_forced: bool,
    },
}

/// The airbrakes estimator. See the module docs for the design and the
/// flight evidence behind it.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct AirbrakesEstimator {
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    state: State,
    config: AirbrakesConfig,
    prev_timestamp_us: Option<u64>,
}

impl AirbrakesEstimator {
    pub fn new(config: AirbrakesConfig) -> Self {
        Self {
            state: State::Armed {
                pad_ring: Deque::new(),
                ignition: IgnitionDetector::new(),
                pad_windows: heapless::Vec::new(),
                window_sum: SVector::zeros(),
                window_n: 0,
                window_elapsed: 0.0,
                calibration: None,
            },
            config,
            prev_timestamp_us: None,
        }
    }

    /// Feed one IMU sample, the time it was taken (us, one monotonic clock)
    /// and the baro altitude ASL (m) from the same instant.
    ///
    /// Returns what the vertical filter's innovation gate did with this
    /// sample's baro reading — returned rather than stored, so there is no
    /// stale value for a later reader to pick up (see
    /// [`crate::BaroGateOutcome`]). Only [`State::AirbrakesEnabled`] runs a gate, so
    /// every other state answers `Accepted`: there is nothing to reject
    /// against before the filter is born.
    pub fn update(
        &mut self,
        timestamp_us: u64,
        imu: &ImuSample,
        altitude_asl: f32,
    ) -> BaroGateOutcome {
        // The very first sample has no predecessor to difference against, so
        // it carries no elapsed time and is stepped by 0. That is not a
        // special case anyone has to reason about: `saturating_sub` plus this
        // clamp already hand out dt = 0 for a duplicate or backwards
        // timestamp, so every integrator, low pass and sustain timer
        // downstream is already required to survive it — and none of them
        // divides by dt. This retired the module's last written-down sample
        // rate (a 1/416 s assumed first step) on 2026-08-17; the phase shift
        // it costs is one sample at the head of the pad calibration windows,
        // measured on LC'25 as a gyro bias of 0.0079287 deg/s against
        // 0.0079378 (Void Lake's 2.0783 did not move at all). Nothing the
        // suite asserts moved: apogee altitudes and birth velocities shift in
        // their fifth significant figure, and the only visible change is
        // which of the MPC's two adjacent extension levels lands on a given
        // tick during the end-of-window dither.
        let dt = match self.prev_timestamp_us {
            Some(prev) => {
                ((timestamp_us.saturating_sub(prev)) as f32 * 1e-6).clamp(0.0, MAX_DT_S)
            }
            None => 0.0,
        };
        self.prev_timestamp_us = Some(timestamp_us);

        let acc = imu.acc;
        let gyro = imu.gyro;

        // Only `Tracking` runs a gate; every other state leaves this at
        // `Accepted`, which is what "no gate to reject anything" means.
        let mut baro_gate = BaroGateOutcome::Accepted;

        match &mut self.state {
            State::Armed {
                pad_ring,
                ignition,
                pad_windows,
                window_sum,
                window_n,
                window_elapsed,
                calibration,
            } => {
                // Run every sample so the low pass and the sustain are
                // already warm when the motor lights; the result is only
                // consulted once the two readiness gates below have passed.
                let accel_says_ignition = ignition.update(
                    Some(acc),
                    dt,
                    self.config.ignition_detection_acc_threshold,
                );

                if pad_ring.is_full() {
                    pad_ring.pop_front();
                }
                let _ = pad_ring.push_back(PadSample {
                    timestamp_us,
                    acc,
                    gyro,
                });
                // Trim to the span, but keep the one sample that has just
                // aged out of it, so the ring always COVERS the span
                // instead of sitting a sample short of it — the rewind
                // replays the whole ring, so a sample short is a span
                // short.
                while pad_ring.len() >= 2 {
                    let second = pad_ring.iter().nth(1).unwrap().timestamp_us;
                    if (timestamp_us.saturating_sub(second)) as f32 * 1e-6 >= PAD_RING_SPAN_S {
                        pad_ring.pop_front();
                    } else {
                        break;
                    }
                }

                // Calibration window collector: finished 2 s windows are
                // kept as candidate (bias, gravity) measurements; the
                // current (possibly ignition-shaken)
                // window is never used. Even a finished window that caught
                // motor spool-up gets rejected by the accel screen.
                let mut sample = SVector::<f32, 6>::zeros();
                sample.fixed_view_mut::<3, 1>(0, 0).copy_from(&acc);
                sample.fixed_view_mut::<3, 1>(3, 0).copy_from(&gyro);
                *window_sum += sample;
                *window_n += 1;
                *window_elapsed += dt;
                if *window_elapsed >= PAD_WINDOW_S {
                    if pad_windows.is_full() {
                        pad_windows.remove(0);
                    }
                    let _ = pad_windows.push(*window_sum / *window_n as f32);
                    *window_sum = SVector::zeros();
                    *window_n = 0;
                    *window_elapsed = 0.0;

                    // Re-screen on every finished window: the calibration
                    // tracks the pad as it is NOW (weather drift, dying
                    // sway) and is ready the instant ignition hits.
                    let was_complete = calibration.is_some();
                    *calibration = screen_pad_windows(pad_windows);
                    if !was_complete && calibration.is_some() {
                        log_info!(
                            "pad calibration complete ({} windows collected)",
                            pad_windows.len()
                        );
                    }
                }

                // There is deliberately no second gate on the ring having
                // filled. It stood here until 2026-08-17 and could not
                // fire: the ring covers its span after PAD_RING_SPAN_S of
                // samples, while the calibration below needs three finished
                // 2 s windows and so cannot exist before 6 s (the first
                // sample carries dt = 0, so the windows span exactly three
                // times PAD_WINDOW_S of measured time).
                // The gate therefore only ever refused samples that the
                // calibration check on the next line was about to refuse
                // anyway. The one input that could separate the two is a
                // clock that runs backwards, where the ring's span
                // collapses while the windows' clamped dt keeps
                // accumulating — and there the gate was actively harmful,
                // blocking ignition detection for as long as the ring took
                // to re-cover its span.
                //
                // Calibration is a hard precondition of ignition
                // detection: without a trustworthy bias and gravity
                // direction there is nothing sane to hand the dead
                // reckoner, so a launch before calibration completes is
                // deliberately NOT detected. `calibration_complete()`
                // surfaces this as an arming/self-test condition — the
                // rocket must not leave the rail before it is true.
                // Both early exits leave the pad state untouched, and no
                // vertical filter exists there, so there is no gate outcome
                // to report but `Accepted`.
                let Some(cal) = *calibration else {
                    return BaroGateOutcome::Accepted;
                };
                if !accel_says_ignition {
                    return BaroGateOutcome::Accepted;
                }
                log_info!("ignition detected, rewinding pad buffer");
                log_info!(
                    "pad calibration: gyro bias {} deg/s",
                    cal.gyro_bias.magnitude().to_degrees()
                );

                // The pad's own mean specific force IS earth UP in the
                // avionics frame — an accelerometer at rest reads +1 g
                // along up — so the pad attitude is a `normalize()` with no
                // rotation to solve and no degenerate case for a mounting
                // that happens to sit exactly inverted.
                let pad_up_av = cal.gravity_av_frame.normalize();
                let mut reckoner = DeadReckoner::new(pad_up_av);

                // Rewind: ignition was detected late (low-pass lag +
                // threshold), so the buffer's tail holds the first moments
                // of real thrust — replay the whole 0.25 s through the dead
                // reckoner. This replay is the ring buffer's ONLY job;
                // gravity and bias both came from the screened windows
                // above.
                let mut prev_t: Option<u64> = None;
                for past in pad_ring.iter() {
                    let past_dt = match prev_t {
                        Some(p) => {
                            ((past.timestamp_us.saturating_sub(p)) as f32 * 1e-6).clamp(0.0, MAX_DT_S)
                        }
                        None => 0.0,
                    };
                    prev_t = Some(past.timestamp_us);
                    if past_dt > 0.0 {
                        // integrating through the still part too is
                        // harmless (rates are ~bias there)
                        reckoner.update(&past.acc, &(past.gyro - cal.gyro_bias), past_dt);
                    }
                }

                log_info!("to stage 1: {:?}", reckoner);
                self.state = State::Stage1 {
                    elapsed: 0.0,
                    acc_sum: Vector3::zeros(),
                    pad_up_av,
                    reckoner,
                    gyro_bias: cal.gyro_bias,
                    ignition_t_us: timestamp_us,
                };
            }

            State::Stage1 {
                elapsed,
                acc_sum,
                pad_up_av,
                reckoner,
                gyro_bias,
                ignition_t_us,
            } => {
                *acc_sum += acc;
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);
                *elapsed += dt;
                if *elapsed < STAGE1_DURATION_S {
                    // Still aligning; no vertical filter, so no gate.
                    return BaroGateOutcome::Accepted;
                }

                // The mean thrust direction IS the airframe axis in the
                // avionics frame, so the burnout latch self-calibrates its
                // mounting and sign from the flight itself — and it is also
                // the whole mounting solution, since tilt is just its angle
                // to the dead reckoner's `up_av`.
                //
                // This `normalize()` is what makes the accumulator's
                // missing division harmless AND what makes the burnout
                // latch — the one magnitude-sensitive consumer, comparing
                // `acc . thrust_axis_av` against -2 m/s^2 — safe: the axis
                // it dots against is unit length by construction, so the
                // sum's arbitrary scale never reaches the threshold.
                let thrust_axis_av = acc_sum.normalize();
                // Measured against the PAD's up, not the reckoner's current
                // one: half a second of gyro integration has already moved
                // the latter, and it is the rail angle this line is for
                // (Void Lake logs 10.2 deg here against the reckoner's
                // 26.1 deg at the same instant). That is `pad_up_av`'s only
                // job, hence the discard — a build with logging compiled
                // out has no other reader for it.
                let _ = &pad_up_av;
                log_info!(
                    "launch angle: {} deg",
                    pad_up_av.angle(&thrust_axis_av).to_degrees()
                );

                self.state = State::DeadReckoning {
                    thrust_axis_av,
                    reckoner: reckoner.clone(),
                    gyro_bias: *gyro_bias,
                    ignition_t_us: *ignition_t_us,
                    baro_ring: Deque::new(),
                    subsonic_sustain: 0.0,
                    drag_lp: None,
                    last_subsonic: false,
                    burnout: false,
                    burnout_sustain: 0.0,
                };
            }

            State::DeadReckoning {
                thrust_axis_av,
                reckoner,
                gyro_bias,
                ignition_t_us,
                baro_ring,
                subsonic_sustain,
                drag_lp,
                last_subsonic,
                burnout,
                burnout_sustain,
            } => {
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);

                // The baro goes in raw.
                if baro_ring.is_full() {
                    baro_ring.pop_front();
                }
                let _ = baro_ring.push_back((timestamp_us, altitude_asl));
                while let Some(front) = baro_ring.front() {
                    if (timestamp_us.saturating_sub(front.0)) as f32 * 1e-6 > BARO_RING_SPAN_S {
                        baro_ring.pop_front();
                    } else {
                        break;
                    }
                }

                // ONE channel, read two ways. In free flight the
                // accelerometer measures specific force, which excludes
                // gravity, so its component along the airframe axis is
                // drag/mass — and under thrust that same component is
                // dominated by the motor and has the opposite sign. Written
                // deceleration-positive, so it reads directly as drag:
                //
                //   a_axial < 0  motor pushing (thrust beats drag)
                //   a_axial > 0  coasting, and the value IS drag/mass
                //
                // This used to be two separate signals — `|acc|` for the
                // drag inversion and `acc . axis` for the burnout latch —
                // which could disagree about what a sample meant, and the
                // magnitude one was the reason the inversion needed guarding
                // in the first place: `|acc|` throws the sign away, so
                // thrust-minus-drag is indistinguishable from pure drag
                // (LC'25 at ignition+6.00 s inverts to a confident, wrong
                // Mach 0.91 while this channel reads -10.66 and "still
                // burning"). Projecting instead of taking the magnitude
                // keeps the sign, so the one channel answers both questions
                // — and `drag_airspeed` rejects a negative `a_drag`, so a
                // thrusting sample no longer inverts to anything at all
                // rather than inverting to a plausible lie.
                let a_axial = -acc.dot(thrust_axis_av);

                // Reading 1 — the burnout latch, on the RAW channel. Raw and
                // not the low pass below, so the 0.3 s sustain is the only
                // lag in it. One-way: motors do not relight, so a noisy
                // sample mid-coast must not be able to re-open the gate.
                if !*burnout {
                    if a_axial > BURNOUT_DECEL_M_S2 {
                        *burnout_sustain += dt;
                        if *burnout_sustain >= BURNOUT_SUSTAIN_S {
                            *burnout = true;
                            log_info!("burnout detected, drag channel is now honest");
                        }
                    } else {
                        *burnout_sustain = 0.0;
                    }
                }

                // Reading 2 — the drag inversion, low-passed because a single
                // raw sample carries the full accelerometer noise and
                // airframe vibration floor.
                let a_drag = match *drag_lp {
                    Some(prev) => prev + (dt / DRAG_LP_TAU_S).min(1.0) * (a_axial - prev),
                    None => a_axial,
                };
                *drag_lp = Some(a_drag);

                let t_since_ignition_s =
                    (timestamp_us.saturating_sub(*ignition_t_us)) as f32 * 1e-6;

                let (born, forced) = match &self.config.mach_lockout {
                    // Subsonic profile: the baro is honest as soon as the
                    // motor is out and we have enough of it buffered for a
                    // clean median. The burnout latch is what stops this
                    // path from handing the MPC a state under thrust —
                    // without it the filter was born ~0.8 s before burnout
                    // (Void Lake: born ignition+0.9 s, burnout +1.7 s).
                    None => (*burnout && ring_span_s(baro_ring) >= 0.25, false),
                    Some(lockout) => {
                        let t_min_s = lockout.earliest_subsonic_after_ignition_us as f32 * 1e-6;
                        let t_max_s = lockout.force_birth_after_ignition_us as f32 * 1e-6;
                        if !*burnout {
                            // NOTHING births before the motor is out — not
                            // the check, and not the T_max backstop either.
                            //
                            // T_max keeps the job it exists for: if the
                            // drag model is wrong and the check never
                            // passes, the axial sign test still latches
                            // (it does not depend on Cd) and the backstop
                            // still fires. It does not cover an
                            // accelerometer dead enough to never show
                            // deceleration — and there the dead reckoner,
                            // the drag check and the KF's own acceleration
                            // input are all equally broken, so staying shut
                            // is the honest outcome.
                            *last_subsonic = false;
                            *subsonic_sustain = 0.0;
                            (false, false)
                        } else if t_since_ignition_s >= t_max_s {
                            log_info!("lockout T_max reached, forced birth");
                            (true, true)
                        } else if t_since_ignition_s < t_min_s {
                            // Before the earliest the sim says we could be
                            // subsonic. Hold the clock at zero so an early
                            // reading cannot bank sustain.
                            *last_subsonic = false;
                            *subsonic_sustain = 0.0;
                            (false, false)
                        } else {
                            // Invert the drag to an airspeed and compare to
                            // the configured Mach. The atmosphere is
                            // evaluated at the CONFIGURED crossing altitude
                            // — a constant from the flight sim, not a
                            // measurement — so the whole exit decision is
                            // independent of both the baro (the sensor it is
                            // deciding about) and of anything that
                            // integrates and can therefore drift. It only
                            // has to be right in the second or so around the
                            // crossing, and that is the altitude the
                            // airframe is at there. See
                            // `MachLockoutConfig::subsonic_crossing_altitude_asl`
                            // for the sensitivity and which way to err.
                            let altitude = lockout.subsonic_crossing_altitude_asl;
                            let subsonic = match drag_airspeed(
                                a_drag,
                                altitude,
                                self.config.rocket.subsonic_cda_over_mass(),
                            ) {
                                Some(airspeed) => {
                                    airspeed
                                        < self.config.max_open_mach
                                            * approximate_speed_of_sound(altitude)
                                }
                                // Nonsensical drag parameter: never pass,
                                // fall through to the T_max backstop.
                                None => false,
                            };
                            *last_subsonic = subsonic;
                            if subsonic {
                                *subsonic_sustain += dt;
                            } else {
                                *subsonic_sustain = 0.0;
                            }
                            (*subsonic_sustain >= SUBSONIC_SUSTAIN_S, false)
                        }
                    }
                };

                // Still dead reckoning, or born on this very sample — either
                // way the gate has not run yet, since the filter fuses its
                // first baro on the NEXT call.
                if !born {
                    return BaroGateOutcome::Accepted;
                }

                // Birth ("born subsonic"): nothing from the garbage period
                // survives into the filter except these two numbers.
                let vv0 = reckoner.vertical_velocity;
                let alt0_asl = match ring_median(baro_ring, timestamp_us, vv0) {
                    Some(m) => m,
                    // no baro at all yet — wait
                    None => return BaroGateOutcome::Accepted,
                };

                // The second Mach test, and the last one: the dead
                // reckoner's own velocity against `max_open_mach` of the
                // local speed of sound. Cd-independent, unlike the drag
                // check that got us here, which is the point — a drag model
                // that overestimates drag reads the inverted airspeed low
                // and passes the check early (measured at Mach 0.887 on an
                // LC'25 replay with a 2x Cd error), and the dead reckoner
                // does not share that error.
                //
                // It lives here, on the way INTO the state, rather than
                // downstream on every sample. Downstream it could withdraw a
                // permission it had already granted, and did: the vertical
                // filter's own birth transient threw its velocity over the
                // limit for 170 ms and shut the brakes again after they had
                // opened. Asked once, of the number the filter is about to
                // be born with, it answers the question it exists for —
                // "is the airframe subsonic enough to open" — and cannot
                // answer it again from a filter that is briefly wrong.
                //
                // The state simply stays here if the test fails: the drag
                // check has already latched, so the next sample retries with
                // a slower rocket. A T_max forced birth waits the same way,
                // which is the intended reading of the backstop — it exists
                // to stop waiting for a broken drag model, not to open the
                // brakes at any speed.
                if vv0 > self.config.max_open_mach * approximate_speed_of_sound(alt0_asl) {
                    return BaroGateOutcome::Accepted;
                }

                let kf = VerticalKF::born(
                    alt0_asl,
                    vv0,
                    if forced {
                        FORCED_BORN_VELOCITY_STD
                    } else {
                        BORN_VELOCITY_STD
                    },
                    KF_Q_ACCEL_STD,
                    KF_R_ALT_STD,
                );
                log_info!(
                    "vertical filter born (forced: {}): alt {}, vv {}",
                    forced,
                    alt0_asl,
                    vv0
                );
                self.state = State::AirbrakesEnabled {
                    thrust_axis_av: *thrust_axis_av,
                    reckoner: reckoner.clone(),
                    gyro_bias: *gyro_bias,
                    kf,
                    born_t_us: timestamp_us,
                    born_forced: forced,
                };
            }

            State::AirbrakesEnabled {
                reckoner,
                gyro_bias,
                kf,
                ..
            } => {
                // The dead reckoner runs for two things here: the attitude
                // behind tilt, and the vertical acceleration the filter
                // predicts with. Its own vertical velocity keeps integrating
                // but nothing reads it past birth, and it no longer carries
                // an altitude at all.
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);

                kf.predict(reckoner.vertical_acceleration, dt);

                // The baro is fused raw, so the dead-reckoned attitude
                // never reaches the altitude or vertical-velocity channel —
                // it survives only as the tilt behind `velocity()`'s
                // horizontal component, and a drifting gyro cannot corrupt
                // what the MPC flies on.
                //
                // Nothing follows this. There is no apogee transition to
                // make: the estimator runs until the wrapper drops it, so
                // the gate outcome is the last thing this state produces.
                baro_gate = kf.update(altitude_asl, dt);
            }
        }

        baro_gate
    }

    /// Altitude ASL from the vertical filter, `None` until it is born.
    ///
    /// Absent — not stale, not integrated — for the whole boost and lockout.
    /// This used to hand out the dead reckoner's doubly-integrated altitude
    /// there, which no consumer needed: the MPC gate cannot be reached before
    /// [`State::AirbrakesEnabled`] anyway, and the log and downlink carry the
    /// deployment half's barometric altitude, which is present in every
    /// state. What the pre-birth value did instead was look like a position
    /// fix while being a drifting open-loop integral nothing corrected.
    pub fn altitude_asl(&self) -> Option<f32> {
        match &self.state {
            State::Armed { .. } | State::Stage1 { .. } | State::DeadReckoning { .. } => None,
            State::AirbrakesEnabled { kf, .. } => Some(kf.altitude_asl()),
        }
    }

    /// Whether this half has latched ignition and left the pad state.
    ///
    /// Deliberately a question about the state machine rather than a side
    /// effect of some value that happens to appear at the same moment: this
    /// used to be read as `launch_pad_altitude_asl().is_some()`, which tied
    /// a timing observation to a number that had no other reader and is now
    /// gone. Note this is THIS half's ignition detector, which runs its own
    /// instance and can latch a sample or two apart from the pyro half's.
    pub fn ignition_latched(&self) -> bool {
        !matches!(self.state, State::Armed { .. })
    }

    /// MPC velocity input: (horizontal, vertical) m/s. Only available once
    /// the vertical filter is running (baro trusted) — which, since
    /// [`State::AirbrakesEnabled`] is the last state, is exactly the window
    /// airbrakes may act in.
    ///
    /// Its sign is also the retirement condition
    /// [`FlightEstimators::update`] reads: a non-positive `y` ends the
    /// airbrakes window. That is the only apogee criterion in the system.
    ///
    /// [`FlightEstimators::update`]: crate::FlightEstimators::update
    pub fn velocity(&self) -> Option<Vector2<f32>> {
        match &self.state {
            State::AirbrakesEnabled {
                kf,
                thrust_axis_av,
                reckoner,
                ..
            } => {
                let vv = kf.vertical_velocity();
                let tilt = axis_tilt(thrust_axis_av, reckoner).min(TILT_CAP_RAD);
                Some(Vector2::new((vv * libm::tanf(tilt)).abs(), vv))
            }
            _ => None,
        }
    }

    /// Rocket axis tilt from vertical, radians (gyro dead reckoning).
    pub fn tilt(&self) -> Option<f32> {
        match &self.state {
            State::DeadReckoning {
                thrust_axis_av,
                reckoner,
                ..
            }
            | State::AirbrakesEnabled {
                thrust_axis_av,
                reckoner,
                ..
            } => Some(axis_tilt(thrust_axis_av, reckoner)),
            _ => None,
        }
    }

    /// True once the axial-sign burnout latch has fired: the motor is out
    /// and the drag channel is honest.
    ///
    /// This is the single condition standing between the estimator and any
    /// chance of the brakes opening — no birth path, check or T_max
    /// backstop, proceeds without it — so it is worth logging per sample.
    /// Without it a flight where the brakes never opened cannot be told
    /// apart from one where the drag check simply never passed.
    ///
    /// `Tracking` implies it, since it cannot be reached otherwise.
    pub fn burnout_detected(&self) -> bool {
        match &self.state {
            State::Armed { .. } | State::Stage1 { .. } => false,
            State::DeadReckoning { burnout, .. } => *burnout,
            State::AirbrakesEnabled { .. } => true,
        }
    }

    /// Which state this half is in — the whole of it, as one value.
    ///
    /// The private [`State`] carries each state's working data; this is its
    /// projection onto the four names, which is what the log stores and what
    /// a caller can compare. Kept as a projection rather than exposing
    /// `State` itself so the working data stays unreachable: everything a
    /// consumer needs from a state is already an accessor, and handing out
    /// the dead reckoner would make that untrue.
    pub fn state(&self) -> AirbrakesState {
        match &self.state {
            State::Armed { .. } => AirbrakesState::Armed,
            State::Stage1 { .. } => AirbrakesState::Stage1,
            State::DeadReckoning { .. } => AirbrakesState::DeadReckoning,
            State::AirbrakesEnabled { .. } => AirbrakesState::AirbrakesEnabled,
        }
    }

    /// True once the brakes may open, and it never goes back to false.
    ///
    /// One question, not the three it used to be spread across. Entering
    /// [`State::AirbrakesEnabled`] means all of: the motor is out, the drag
    /// check (or the T_max backstop) has passed, the vertical filter exists
    /// and is fusing the baro, and the airframe was under `max_open_mach`
    /// when it did. There is no state after this one and no way back to the
    /// ones before it, so a caller that has seen this true does not have to
    /// ask again — which is exactly what the MPC's old per-sample gate was
    /// doing, and what let a filter transient close a permission that had
    /// already been granted.
    pub fn airbrakes_enabled(&self) -> bool {
        matches!(self.state, State::AirbrakesEnabled { .. })
    }

    /// The lockout-exit drag check, for logging/telemetry: whether the
    /// drag-inverted airspeed is currently below the airframe's configured
    /// `max_open_mach`. `None` outside the dead-reckoning phase, and always
    /// `false` before `t_min_us` (the check is not consulted while the motor
    /// may still be burning).
    pub fn subsonic_by_drag(&self) -> Option<bool> {
        match &self.state {
            State::DeadReckoning { last_subsonic, .. } => Some(*last_subsonic),
            _ => None,
        }
    }

    /// When and how the vertical filter was born: (timestamp_us, forced).
    /// `forced` means the T_max ceiling fired instead of the check.
    pub fn birth(&self) -> Option<(u64, bool)> {
        match &self.state {
            State::AirbrakesEnabled {
                born_t_us,
                born_forced,
                ..
            } => Some((*born_t_us, *born_forced)),
            _ => None,
        }
    }

    /// True once the pad screening has produced a trustworthy calibration:
    /// at least `MIN_CALIBRATION_WINDOWS` (3) finished 2 s windows each
    /// read 1 g and no net rotation, i.e. 6 s of data taken while the
    /// airframe was demonstrably on the rail. The rocket sits armed for
    /// minutes, so this costs nothing operationally.
    ///
    /// While this is false the estimator REFUSES to detect ignition (there
    /// is no fallback calibration to fly on), so it must be surfaced as an
    /// arming/self-test condition. What holds it false is a pad that was
    /// never observed — a recorder started too late, or an airframe still
    /// being handled — and not merely a windy one: sway degrades the
    /// numbers slightly rather than withholding them, which at hundredths
    /// of a deg/s against a 5 deg tilt budget is the right trade.
    pub fn calibration_complete(&self) -> bool {
        match &self.state {
            State::Armed { calibration, .. } => calibration.is_some(),
            // Ignition can only have been detected with a complete
            // calibration, so every later state implies it.
            _ => true,
        }
    }

    /// The airframe's subsonic Mach, as configured — the same value this
    /// estimator's own drag check votes at. Read by the MPC gate in
    /// [`FlightEstimators`], which has no config of its own.
    ///
    /// [`FlightEstimators`]: crate::FlightEstimators
    pub fn max_open_mach(&self) -> f32 {
        self.config.max_open_mach
    }
}

/// Screen the finished pad windows and, if enough of them qualify,
/// produce the pad calibration: keep the windows during which the
/// airframe was demonstrably sitting on a rail, and average them.
///
/// Returns `None` until at least `MIN_CALIBRATION_WINDOWS` windows
/// qualify. There is no fallback: too few windows is not-launch-ready,
/// never a silent guess.
///
/// Note what this deliberately does NOT do: reject a window for
/// disagreeing with the other windows. See the constants above for the
/// measurement that retired that, and for why the disagreement test was
/// the wrong shape for the one job it was doing.
fn screen_pad_windows(
    windows: &heapless::Vec<SVector<f32, 6>, MAX_PAD_WINDOWS>,
) -> Option<PadCalibration> {
    if windows.len() < MIN_CALIBRATION_WINDOWS {
        return None;
    }

    // A rocket on a rail reads 1 g and is not turning. Nothing else about
    // a pad window is knowable in absolute terms — the gyro bias and the
    // pad orientation are precisely what this function exists to measure
    // — so nothing else is asserted. A baro channel used to ride along in
    // these windows, unscreened for the same reason (a pressure transient
    // is not evidence that the airframe moved); it is gone, and the pad
    // altitude now has exactly one owner, the deployment half.
    let on_the_pad = |w: &SVector<f32, 6>| -> bool {
        let acc: Vector3<f32> = w.fixed_view::<3, 1>(0, 0).into();
        let gyro: Vector3<f32> = w.fixed_view::<3, 1>(3, 0).into();
        (acc.magnitude() - 9.81).abs() <= PAD_GRAVITY_TOLERANCE_M_S2
            && gyro.magnitude() <= PAD_ROTATION_LIMIT_RAD_S
    };

    let mut sum = SVector::<f32, 6>::zeros();
    let mut n = 0usize;
    for w in windows.iter().filter(|w| on_the_pad(w)) {
        sum += w;
        n += 1;
    }
    if n < MIN_CALIBRATION_WINDOWS {
        return None;
    }
    let mean = sum / n as f32;

    Some(PadCalibration {
        gyro_bias: mean.fixed_view::<3, 1>(3, 0).into(),
        gravity_av_frame: mean.fixed_view::<3, 1>(0, 0).into(),
    })
}

fn ring_span_s(ring: &Deque<(u64, f32), BARO_RING_CAP>) -> f32 {
    match (ring.front(), ring.back()) {
        (Some(front), Some(back)) => (back.0.saturating_sub(front.0)) as f32 * 1e-6,
        _ => 0.0,
    }
}

/// Airspeed (m/s) implied by the measured drag deceleration, inverting
/// `a = 0.5 * rho * v^2 * cda_over_mass`. `None` if the atmosphere or the
/// drag parameter is degenerate, which makes a misconfigured airframe fail
/// toward "never pass" rather than toward "always subsonic".
fn drag_airspeed(a_drag: f32, altitude_asl: f32, cda_over_mass: f32) -> Option<f32> {
    let rho = approximate_air_density(altitude_asl);
    if !(cda_over_mass > 0.0) || !(rho > 0.0) || !(a_drag >= 0.0) {
        return None;
    }
    Some(libm::sqrtf(2.0 * a_drag / (rho * cda_over_mass)))
}

/// Median of 9 samples spaced evenly across the ring, each carried forward
/// to `now_us` at `vertical_velocity` — a transient can't move it, unlike a
/// mean or a single reading, and unlike a plain median it is not stale.
///
/// The carry is the whole point and it was missing until 2026-08-17. Nine
/// picks evenly spaced across the ring make the median the MIDDLE pick, i.e.
/// the altitude the rocket had half a ring-span ago: `BARO_RING_SPAN_S / 2`
/// = 0.35 s, which at the 224 m/s this is called at is **80 m low**. The
/// filter was then born believing an altitude it had left a third of a
/// second earlier, while `p00` said it knew that altitude to 3 m — 27 sigma
/// wrong and confident about it. The baro's standing +80 m innovation got
/// worked off partly through the velocity channel, which on the Void Lake
/// replay peaked at +62 m/s (285 against a true 224) 84 ms after birth and
/// took half a second to bleed off. The Mach gate saw that excursion and
/// shut the brakes again; when it reopened the MPC's velocity input was
/// still 12% high.
///
/// Carrying each pick forward removes the lag exactly for a constant-velocity
/// climb, and leaves only the curvature over half a ring span — at -4.5 m/s^2
/// and 0.35 s, about 0.3 m. It is done per pick rather than once on the
/// median so that the median still ranks readings that are comparable: at
/// 224 m/s the raw picks span 157 m, which is fifty times the baro noise the
/// median is there to reject, and ranking them ranks time rather than
/// plausibility.
fn ring_median(
    ring: &Deque<(u64, f32), BARO_RING_CAP>,
    now_us: u64,
    vertical_velocity: f32,
) -> Option<f32> {
    let n = ring.len();
    if n == 0 {
        return None;
    }
    let mut picks = [0.0f32; 9];
    let mut count = 0usize;
    for (i, (t_us, alt)) in ring.iter().enumerate() {
        while count < 9 && i == (count * (n - 1)) / 8 {
            let age_s = (now_us.saturating_sub(*t_us)) as f32 * 1e-6;
            picks[count] = *alt + vertical_velocity * age_s;
            count += 1;
        }
    }
    let picks = &mut picks[..count];
    // `total_cmp`, not `partial_cmp(..).unwrap()`: the altitude reaches this
    // ring unguarded, and one NaN among the nine picks used to panic the
    // whole estimator on the sample that ends the Mach lockout. The upstream
    // source of non-finite altitudes is fixed (VLF5 `sensor_tasks.rs` rejects
    // non-finite and non-positive pressure), so this is the second layer, and
    // it costs a token rather than a mechanism. A NaN now simply sorts to one
    // end and the median stays a real reading.
    picks.sort_unstable_by(f32::total_cmp);
    Some(picks[count / 2])
}

/// Rocket-axis tilt from vertical (radians): the angle between the airframe
/// axis and earth UP, both written in the avionics frame — which is the
/// frame both are already in, so no rotation is applied to take it.
fn axis_tilt(thrust_axis_av: &Vector3<f32>, reckoner: &DeadReckoner) -> f32 {
    reckoner.up_av.angle(thrust_axis_av)
}
