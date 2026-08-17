use heapless::Deque;
use nalgebra::{SVector, UnitQuaternion, UnitVector3, Vector2, Vector3};

use crate::{
    airbrakes_estimator::{
        AirbrakesConfig, MAX_DT_S, Measurement, NOMINAL_DT, dead_reckoner::DeadReckoner,
        vertical_kf::VerticalKF, welford::Welford,
    },
    baro_gate::BaroGateOutcome,
    utils::{approximate_air_density, approximate_speed_of_sound},
};

const UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

// --- Pad calibration (Piece 1) ---------------------------------------------
// Everything flight needs to learn while the rocket is still — gyro bias,
// gravity direction (pad orientation), and pad altitude — comes from ONE
// structure: finished 2 s window means (gyro + accel + baro together),
// averaged over the windows that look like a rocket sitting on a rail.
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
// budget, and moved the pad altitude 0.1 m the WRONG way. Deleting it
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
const PAD_RING_SPAN_S: f32 = 2.0;
/// Capacity for that span. Trimming is by time, not by count, so this only
/// has to be large enough that a faster-than-nominal sensor still buffers
/// the whole span instead of quietly rewinding less of it: 1024 covers 2 s
/// at up to 512 Hz. Past that the ring saturates and the rewind gets
/// shorter, which is exactly what the old count-based buffer did at ANY
/// rate above nominal — degraded, not broken (see the readiness gate in
/// `State::OnPad`).
const PAD_RING_CAP: usize = 1024;
/// Time constant of the low pass in front of the ignition threshold,
/// a 10 Hz corner (`1 / 2*pi*10`).
///
/// This used to be a 2nd-order Butterworth biquad *designed at the nominal
/// sample rate* — the last thing in this estimator that assumed one. A
/// one-pole driven by the measured dt has the same nominal corner and no
/// rate assumption at all, and it is the same idiom as `drag_lp` below.
/// Order is not what this filter is buying: the threshold is 4 g against a
/// 0.04 m/s^2 pad noise floor, three orders of magnitude of margin, so
/// what it actually rejects is a knock on the rail.
///
/// The swap also fixed something the old filter was hiding. Measured on
/// the Osiris replay, where the motor takes the axial channel from 12 to
/// 83 m/s^2 in 20 ms: the biquad's output had moved 9.81 -> 11.06 over
/// that whole ramp — nothing like a 10 Hz filter, which is where its
/// ~65 ms of detection lag came from. Ignition detection now lands at
/// ignition+24 ms instead of ignition+77 ms.
///
/// That matters beyond the detector, because detection is the ORIGIN every
/// lockout timer is measured from ([`MachLockoutConfig`]), while the
/// constants themselves come from the simulation's time-since-ignition. A
/// detector that lags biases all of them late by exactly that lag, so
/// removing it is what makes those constants mean what they say. It also
/// spends 53 ms of the drag check's margin, which the 1 s sustain covers
/// with ~0.9 s to spare; see `clipped_accel_still_flies_the_profile`.
///
/// [`MachLockoutConfig`]: crate::airbrakes_estimator::MachLockoutConfig
const IGNITION_LP_TAU_S: f32 = 0.0159;
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
/// Burnout is latched when the axial specific force has been at least this
/// negative continuously for `BURNOUT_SUSTAIN_S`.
///
/// The SIGN is the discriminator, not the magnitude: thrust acts along
/// +axis and drag along -axis, so the axial channel crosses zero at burnout
/// and stays negative for the whole coast. A magnitude test cannot tell
/// 11.7 m/s^2 of thrust-minus-drag from 11.7 m/s^2 of pure drag — measured
/// on LC'25 at ignition+6.00 s, where inverting |accel| yields a confident
/// and completely wrong Mach 0.91 while the axial channel still reads
/// +10.66 and correctly says "still burning".
///
/// -2 m/s^2 rather than 0 keeps the latch clear of the crossing itself.
/// Coast drag is 7-21 m/s^2 over the region that matters, and the pad noise
/// floor is ~0.04 m/s^2, so there is no contest.
const BURNOUT_AXIAL_M_S2: f32 = -2.0;
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
const BARO_RING_SPAN_S: f32 = 0.7;

// --- Birth of the vertical filter (Piece 4) -------------------------------
/// Initial vertical-velocity uncertainty at a check-approved birth (the
/// dead-reckoned velocity is trusted to roughly this, m/s std).
const BORN_VELOCITY_STD: f32 = 15.0;
/// ...and at a forced (T_max) birth, where the dead reckoner may have
/// drifted badly: bigger, so the baro pulls velocity back fast.
const FORCED_BORN_VELOCITY_STD: f32 = 30.0;
const KF_Q_ACCEL_STD: f32 = 0.5;
const KF_R_ALT_STD: f32 = 3.0;

// --- Apogee latch ----------------------------------------------------------
const APOGEE_VV_M_S: f32 = 1.0;
const APOGEE_SUSTAIN_S: f32 = 0.5;
/// For the latch the baro must be alive: an accepted update within this
/// long...
const BARO_FRESH_S: f32 = 0.25;
/// ...and not frozen (identical raw readings) for longer than this.
const BARO_FROZEN_S: f32 = 0.5;

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
    /// Mean baro altitude of the surviving windows.
    launch_pad_altitude_asl: f32,
}

/// One buffered pre-ignition sample: everything the rewind replays, and
/// nothing else. The baro altitude is deliberately absent — the rewind
/// only drives the dead reckoner, and the pad altitude it starts from
/// comes from the screened windows. At 32 bytes against `Measurement`'s
/// 40, dropping it pays for the ring's capacity headroom outright.
#[derive(Debug, Clone, Copy)]
struct PadSample {
    timestamp_us: u64,
    acc: Vector3<f32>,
    gyro: Vector3<f32>,
}

#[derive(Debug)]
enum State {
    /// Stable on pad: collect screened calibration windows (gyro bias,
    /// gravity direction, pad altitude), and watch for ignition with a
    /// 2 s rolling buffer that is rewound once ignition is detected. The
    /// rolling buffer's ONLY job is that rewind — calibration comes
    /// entirely from the windows.
    OnPad {
        pad_ring: Deque<PadSample, PAD_RING_CAP>,
        /// Low-passed accelerometer feeding the ignition threshold. `None`
        /// until the first sample.
        acc_lp: Option<Vector3<f32>>,
        /// Finished window means, packed like `Measurement`: acc[0..3],
        /// gyro[3..6], baro altitude [6].
        pad_windows: heapless::Vec<SVector<f32, 7>, MAX_PAD_WINDOWS>,
        window_welford: Welford<7>,
        window_elapsed: f32,
        /// Latest screening result; `None` until enough windows agree.
        /// Ignition detection is refused while this is `None`.
        calibration: Option<PadCalibration>,
    },

    /// First half second of powered flight: the thrust direction tells us
    /// how the avionics are mounted in the rocket.
    Stage1 {
        elapsed: f32,
        acc_welford: Welford<3>,
        pad_av_orientation: UnitQuaternion<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        launch_pad_altitude_asl: f32,
        ignition_t_us: u64,
    },

    /// Boost and Mach lockout: inertial dead reckoning only, no filter.
    /// The baro is buffered (to pick a birth altitude) but never fused.
    DeadReckoning {
        q_av_to_rocket: UnitQuaternion<f32>,
        /// Unit airframe axis in the avionics frame, from the stage-1 mean
        /// thrust direction. Thrust is +, drag is -.
        thrust_axis_av: Vector3<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        launch_pad_altitude_asl: f32,
        ignition_t_us: u64,
        /// (timestamp_us, raw baro altitude)
        baro_ring: Deque<(u64, f32), BARO_RING_CAP>,
        subsonic_sustain: f32,
        /// Low-passed accelerometer magnitude — drag/mass once the motor
        /// is out. `None` until the first sample of this state.
        drag_lp: Option<f32>,
        last_subsonic: bool,
        /// Latched once the axial channel proves the motor is out. Nothing
        /// downstream — neither the drag check nor the subsonic birth — is
        /// allowed to proceed before it.
        burnout: bool,
        burnout_sustain: f32,
    },

    /// The baro is honest: the 2-state vertical filter exists and runs to
    /// apogee. Tilt still comes from the gyro dead reckoner.
    Tracking {
        q_av_to_rocket: UnitQuaternion<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        launch_pad_altitude_asl: f32,
        kf: VerticalKF,
        born_t_us: u64,
        born_forced: bool,
        apogee_sustain: f32,
        baro_accept_age: f32,
        last_raw_baro: f32,
        baro_frozen_s: f32,
    },

    Apogee {
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
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
    /// What the vertical filter's innovation gate did with the sample this
    /// estimator last processed. `Accepted` before the filter is born and
    /// after apogee, when there is no gate running.
    last_baro_gate: BaroGateOutcome,
}

impl AirbrakesEstimator {
    pub fn new(config: AirbrakesConfig) -> Self {
        Self {
            state: State::OnPad {
                pad_ring: Deque::new(),
                acc_lp: None,
                pad_windows: heapless::Vec::new(),
                window_welford: Welford::new(),
                window_elapsed: 0.0,
                calibration: None,
            },
            config,
            prev_timestamp_us: None,
            last_baro_gate: BaroGateOutcome::Accepted,
        }
    }

    /// Feed one timestamped IMU+baro sample.
    pub fn update(&mut self, z: &Measurement) {
        let dt = match self.prev_timestamp_us {
            Some(prev) => {
                ((z.timestamp_us.saturating_sub(prev)) as f32 * 1e-6).clamp(0.0, MAX_DT_S)
            }
            None => NOMINAL_DT,
        };
        self.prev_timestamp_us = Some(z.timestamp_us);

        let acc = z.acceleration();
        let gyro = z.angular_velocity();

        // Only `Tracking` runs a gate; every other state leaves this at
        // `Accepted`, which is what "no gate to reject anything" means.
        let mut baro_gate = BaroGateOutcome::Accepted;

        match &mut self.state {
            State::OnPad {
                pad_ring,
                acc_lp,
                pad_windows,
                window_welford,
                window_elapsed,
                calibration,
            } => {
                // Ignition low pass: one pole on the measured dt. Clamped
                // at alpha = 1 so a long stall snaps to the sample rather
                // than overshooting past it.
                let acc_low_passed = match *acc_lp {
                    Some(prev) => prev + (dt / IGNITION_LP_TAU_S).min(1.0) * (acc - prev),
                    None => acc,
                };
                *acc_lp = Some(acc_low_passed);

                if pad_ring.is_full() {
                    pad_ring.pop_front();
                }
                let _ = pad_ring.push_back(PadSample {
                    timestamp_us: z.timestamp_us,
                    acc,
                    gyro,
                });
                // Trim to the span, but keep the one sample that has just
                // aged out of it, so the ring always COVERS the span
                // instead of sitting a sample short of it — which is what
                // lets the readiness gate below be an honest `>=`.
                while pad_ring.len() >= 2 {
                    let second = pad_ring.iter().nth(1).unwrap().timestamp_us;
                    if (z.timestamp_us.saturating_sub(second)) as f32 * 1e-6 >= PAD_RING_SPAN_S {
                        pad_ring.pop_front();
                    } else {
                        break;
                    }
                }

                // Calibration window collector: finished 2 s windows are
                // kept as candidate (bias, gravity, pad-altitude)
                // measurements; the current (possibly ignition-shaken)
                // window is never used. Even a finished window that caught
                // motor spool-up gets rejected by the accel screen.
                let mut sample = SVector::<f32, 7>::zeros();
                sample.fixed_view_mut::<3, 1>(0, 0).copy_from(&acc);
                sample.fixed_view_mut::<3, 1>(3, 0).copy_from(&gyro);
                sample[6] = z.altitude_asl();
                window_welford.update(&sample);
                *window_elapsed += dt;
                if *window_elapsed >= PAD_WINDOW_S {
                    if pad_windows.is_full() {
                        pad_windows.remove(0);
                    }
                    let _ = pad_windows.push(window_welford.mean());
                    *window_welford = Welford::new();
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

                // Readiness: the rewind must have a full span to replay
                // before ignition may be detected. `is_full` is the
                // degraded path — at a sample rate high enough to saturate
                // the ring before the span fills, detection still happens,
                // just with a shorter rewind. That is what the old
                // count-based buffer did at every rate; here it is the
                // fallback rather than the rule.
                if !pad_ring.is_full() && pad_ring_span_s(pad_ring) < PAD_RING_SPAN_S {
                    return;
                }
                // Calibration is a hard precondition of ignition
                // detection: without a trustworthy bias, gravity direction
                // and pad altitude there is nothing sane to hand the dead
                // reckoner, so a launch before calibration completes is
                // deliberately NOT detected. `calibration_complete()`
                // surfaces this as an arming/self-test condition — the
                // rocket must not leave the rail before it is true.
                let Some(cal) = *calibration else {
                    return;
                };
                let threshold = self.config.ignition_detection_acc_threshold;
                if acc_low_passed.magnitude_squared() <= threshold * threshold {
                    return;
                }
                log_info!("ignition detected, rewinding pad buffer");
                log_info!(
                    "pad calibration: gyro bias {} deg/s, pad altitude {} m",
                    cal.gyro_bias.magnitude().to_degrees(),
                    cal.launch_pad_altitude_asl
                );

                let pad_av_orientation =
                    quaternion_from_start_and_end_vector(&UP, &cal.gravity_av_frame);
                let mut reckoner = DeadReckoner::new(pad_av_orientation);
                reckoner.position.z = cal.launch_pad_altitude_asl;

                // Rewind: ignition was detected late (low-pass lag +
                // threshold), so the buffer's tail holds the first moments
                // of real thrust — replay the whole 2 s through the dead
                // reckoner. This replay is the ring buffer's ONLY job;
                // gravity, pad altitude and bias all came from the
                // screened windows above.
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
                    acc_welford: Welford::new(),
                    pad_av_orientation,
                    reckoner,
                    gyro_bias: cal.gyro_bias,
                    launch_pad_altitude_asl: cal.launch_pad_altitude_asl,
                    ignition_t_us: z.timestamp_us,
                };
            }

            State::Stage1 {
                elapsed,
                acc_welford,
                pad_av_orientation,
                reckoner,
                gyro_bias,
                launch_pad_altitude_asl,
                ignition_t_us,
            } => {
                acc_welford.update(&acc);
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);
                *elapsed += dt;
                if *elapsed < STAGE1_DURATION_S {
                    return;
                }

                // The mean thrust direction (in the earth frame, via the
                // pad orientation) is the rocket's axis: this calibrates
                // how the avionics are mounted in the airframe.
                let avg_acc_av_frame = acc_welford.mean();
                let avg_acc_earth_frame = pad_av_orientation.transform_vector(&avg_acc_av_frame);
                log_info!(
                    "launch angle: {} deg",
                    UP.angle(&avg_acc_earth_frame).to_degrees()
                );
                let q_earth_to_rocket =
                    quaternion_from_start_and_end_vector(&avg_acc_earth_frame, &UP);
                let q_av_to_rocket = pad_av_orientation.inverse() * q_earth_to_rocket;

                self.state = State::DeadReckoning {
                    q_av_to_rocket,
                    // The mean thrust direction IS the airframe axis in the
                    // avionics frame, so the burnout latch self-calibrates
                    // its mounting and sign from the flight itself.
                    thrust_axis_av: avg_acc_av_frame.normalize(),
                    reckoner: reckoner.clone(),
                    gyro_bias: *gyro_bias,
                    launch_pad_altitude_asl: *launch_pad_altitude_asl,
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
                q_av_to_rocket,
                thrust_axis_av,
                reckoner,
                gyro_bias,
                launch_pad_altitude_asl,
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
                let _ = baro_ring.push_back((z.timestamp_us, z.altitude_asl()));
                while let Some(front) = baro_ring.front() {
                    if (z.timestamp_us.saturating_sub(front.0)) as f32 * 1e-6 > BARO_RING_SPAN_S {
                        baro_ring.pop_front();
                    } else {
                        break;
                    }
                }

                // Drag channel: in free flight the accelerometer measures
                // specific force, which excludes gravity, so its magnitude
                // is drag/mass. Low-passed because it is a single raw
                // sample carrying the full noise and vibration floor.
                let a_drag = match *drag_lp {
                    Some(prev) => prev + (dt / DRAG_LP_TAU_S).min(1.0) * (acc.magnitude() - prev),
                    None => acc.magnitude(),
                };
                *drag_lp = Some(a_drag);

                // Burnout, measured rather than timed: the axial channel is
                // strongly positive under thrust and negative in free
                // flight, so a sustained negative reading proves the motor
                // is out. One-way latch — motors do not relight.
                if !*burnout {
                    if acc.dot(thrust_axis_av) < BURNOUT_AXIAL_M_S2 {
                        *burnout_sustain += dt;
                        if *burnout_sustain >= BURNOUT_SUSTAIN_S {
                            *burnout = true;
                            log_info!("burnout detected, drag channel is now honest");
                        }
                    } else {
                        *burnout_sustain = 0.0;
                    }
                }

                let t_since_ignition_s =
                    (z.timestamp_us.saturating_sub(*ignition_t_us)) as f32 * 1e-6;

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
                            // the configured Mach. Air density comes from
                            // the DEAD
                            // RECKONED altitude, not the baro, so the whole
                            // exit decision is independent of the sensor it
                            // is deciding about. (Either source works —
                            // measured on LC'25 they differ by <=0.01 Mach
                            // — but taking the baro out entirely removes
                            // the question.)
                            let altitude = reckoner.position.z;
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

                if !born {
                    return;
                }

                // Birth ("born subsonic"): nothing from the garbage period
                // survives into the filter except these two numbers.
                let alt0_asl = match ring_median(baro_ring) {
                    Some(m) => m,
                    None => return, // no baro at all yet — wait
                };
                let vv0 = reckoner.velocity.z;
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
                self.state = State::Tracking {
                    q_av_to_rocket: *q_av_to_rocket,
                    reckoner: reckoner.clone(),
                    gyro_bias: *gyro_bias,
                    launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    kf,
                    born_t_us: z.timestamp_us,
                    born_forced: forced,
                    apogee_sustain: 0.0,
                    baro_accept_age: 0.0,
                    last_raw_baro: z.altitude_asl(),
                    baro_frozen_s: 0.0,
                };
            }

            State::Tracking {
                reckoner,
                gyro_bias,
                launch_pad_altitude_asl,
                kf,
                apogee_sustain,
                baro_accept_age,
                last_raw_baro,
                baro_frozen_s,
                ..
            } => {
                // The dead reckoner runs for tilt only (gyro-only
                // orientation); its velocity/position are unused here.
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);

                kf.predict(reckoner.acceleration.z, dt);

                // The baro is fused raw, so the dead-reckoned attitude
                // never reaches the altitude or vertical-velocity channel —
                // it survives only as the tilt behind `velocity()`'s
                // horizontal component, and a drifting gyro cannot corrupt
                // what the MPC flies on.
                // Only a plain `Accepted` counts as fresh baro: a resync
                // snaps altitude but does not fuse the sample, which is what
                // this age has always meant.
                baro_gate = kf.update(z.altitude_asl(), dt);
                if baro_gate == BaroGateOutcome::Accepted {
                    *baro_accept_age = 0.0;
                } else {
                    *baro_accept_age += dt;
                }

                // Frozen-baro watch: a dead sensor repeats its last value
                // exactly. A live baro at this noise level never does for
                // long.
                if z.altitude_asl() == *last_raw_baro {
                    *baro_frozen_s += dt;
                } else {
                    *baro_frozen_s = 0.0;
                    *last_raw_baro = z.altitude_asl();
                }

                // Apogee latches only with persistence AND a healthy baro
                // (red-team finding: a frozen/offset baro plus a
                // single-sample latch permanently kills the airbrakes).
                let baro_healthy =
                    *baro_accept_age < BARO_FRESH_S && *baro_frozen_s < BARO_FROZEN_S;
                if kf.vertical_velocity() < APOGEE_VV_M_S && baro_healthy {
                    *apogee_sustain += dt;
                } else {
                    *apogee_sustain = 0.0;
                }
                if *apogee_sustain >= APOGEE_SUSTAIN_S {
                    log_info!("apogee latched at {}", kf.altitude_asl());
                    self.state = State::Apogee {
                        altitude_asl: kf.altitude_asl(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }

            State::Apogee { .. } => {}
        }

        self.last_baro_gate = baro_gate;
    }

    /// Best current altitude ASL: the filter once born, dead reckoning
    /// before that, `None` on the pad.
    pub fn altitude_asl(&self) -> Option<f32> {
        match &self.state {
            State::OnPad { .. } => None,
            State::Stage1 { reckoner, .. } | State::DeadReckoning { reckoner, .. } => {
                Some(reckoner.position.z)
            }
            State::Tracking { kf, .. } => Some(kf.altitude_asl()),
            State::Apogee { altitude_asl, .. } => Some(*altitude_asl),
        }
    }

    pub fn launch_pad_altitude_asl(&self) -> Option<f32> {
        match &self.state {
            State::OnPad { .. } => None,
            State::Stage1 {
                launch_pad_altitude_asl,
                ..
            }
            | State::DeadReckoning {
                launch_pad_altitude_asl,
                ..
            }
            | State::Tracking {
                launch_pad_altitude_asl,
                ..
            }
            | State::Apogee {
                launch_pad_altitude_asl,
                ..
            } => Some(*launch_pad_altitude_asl),
        }
    }

    /// MPC velocity input: (horizontal, vertical) m/s. Only available while
    /// the vertical filter is running (baro trusted, before apogee) — which
    /// is exactly the window the airbrakes may act in.
    pub fn velocity(&self) -> Option<Vector2<f32>> {
        match &self.state {
            State::Tracking {
                kf,
                q_av_to_rocket,
                reckoner,
                ..
            } => {
                let vv = kf.vertical_velocity();
                let tilt = axis_tilt(q_av_to_rocket, reckoner).min(TILT_CAP_RAD);
                Some(Vector2::new((vv * libm::tanf(tilt)).abs(), vv))
            }
            _ => None,
        }
    }

    /// Rocket axis tilt from vertical, radians (gyro dead reckoning).
    pub fn tilt(&self) -> Option<f32> {
        match &self.state {
            State::DeadReckoning {
                q_av_to_rocket,
                reckoner,
                ..
            }
            | State::Tracking {
                q_av_to_rocket,
                reckoner,
                ..
            } => Some(axis_tilt(q_av_to_rocket, reckoner)),
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
    /// The later states imply it, since neither can be reached otherwise.
    pub fn burnout_detected(&self) -> bool {
        match &self.state {
            State::OnPad { .. } | State::Stage1 { .. } => false,
            State::DeadReckoning { burnout, .. } => *burnout,
            State::Tracking { .. } | State::Apogee { .. } => true,
        }
    }

    /// True once the vertical filter exists (the baro is trusted). The
    /// airbrakes gate requires this.
    pub fn baro_trusted(&self) -> bool {
        matches!(
            self.state,
            State::Tracking { .. } | State::Apogee { .. }
        )
    }

    pub fn is_apogee(&self) -> bool {
        matches!(self.state, State::Apogee { .. })
    }

    /// What the vertical filter's innovation gate did with the sample this
    /// estimator last processed. Read it immediately after [`Self::update`]:
    /// it describes that one sample and is overwritten by the next.
    ///
    /// Only the vertical filter has a gate, so this is `Accepted` before the
    /// filter is born and after apogee — there is nothing to reject against
    /// in either case.
    pub fn baro_gate(&self) -> BaroGateOutcome {
        self.last_baro_gate
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
            State::Tracking {
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
            State::OnPad { calibration, .. } => calibration.is_some(),
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
    windows: &heapless::Vec<SVector<f32, 7>, MAX_PAD_WINDOWS>,
) -> Option<PadCalibration> {
    if windows.len() < MIN_CALIBRATION_WINDOWS {
        return None;
    }

    // A rocket on a rail reads 1 g and is not turning. Nothing else about
    // a pad window is knowable in absolute terms — the gyro bias and the
    // pad orientation are precisely what this function exists to measure
    // — so nothing else is asserted. In particular the baro channel is
    // unscreened: a pressure transient is not evidence that the airframe
    // moved, and the old unit-screen let one throw away good gyro data.
    let on_the_pad = |w: &SVector<f32, 7>| -> bool {
        let acc: Vector3<f32> = w.fixed_view::<3, 1>(0, 0).into();
        let gyro: Vector3<f32> = w.fixed_view::<3, 1>(3, 0).into();
        (acc.magnitude() - 9.81).abs() <= PAD_GRAVITY_TOLERANCE_M_S2
            && gyro.magnitude() <= PAD_ROTATION_LIMIT_RAD_S
    };

    let mut sum = SVector::<f32, 7>::zeros();
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
        launch_pad_altitude_asl: mean[6],
    })
}

/// Measured time from the oldest to the newest buffered pad sample.
fn pad_ring_span_s(ring: &Deque<PadSample, PAD_RING_CAP>) -> f32 {
    match (ring.front(), ring.back()) {
        (Some(front), Some(back)) => {
            (back.timestamp_us.saturating_sub(front.timestamp_us)) as f32 * 1e-6
        }
        _ => 0.0,
    }
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

/// Median of 9 samples spaced evenly across the ring — a transient can't
/// move it, unlike a mean or a single reading.
fn ring_median(ring: &Deque<(u64, f32), BARO_RING_CAP>) -> Option<f32> {
    let n = ring.len();
    if n == 0 {
        return None;
    }
    let mut picks = [0.0f32; 9];
    let mut count = 0usize;
    for (i, (_, alt)) in ring.iter().enumerate() {
        while count < 9 && i == (count * (n - 1)) / 8 {
            picks[count] = *alt;
            count += 1;
        }
    }
    let picks = &mut picks[..count];
    picks.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    Some(picks[count / 2])
}

fn axis_tilt(q_av_to_rocket: &UnitQuaternion<f32>, reckoner: &DeadReckoner) -> f32 {
    let rocket_orientation = reckoner.orientation * *q_av_to_rocket;
    UP.angle(&rocket_orientation.transform_vector(&UP))
}

/// returns a passive rotation quaternion that would rotate start vector to
/// end vector: the frame rotation under which coordinates `start` become
/// `end`. Operationally (nalgebra active semantics) that means
/// `q.transform_vector(end) == start` — e.g.
/// `quaternion_from_start_and_end_vector(&UP, &gravity_av)` maps the
/// device-frame gravity vector onto earth UP, which is exactly the
/// device->earth attitude the `DeadReckoner` wants.
fn quaternion_from_start_and_end_vector(
    start: &Vector3<f32>,
    end: &Vector3<f32>,
) -> UnitQuaternion<f32> {
    let start = start.normalize();
    let end = end.normalize();

    let axis = UnitVector3::new_normalize(end.cross(&start));
    let angle = end.angle(&start);

    if angle.to_degrees() < 0.05 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_axis_angle(&axis, angle)
    }
}
