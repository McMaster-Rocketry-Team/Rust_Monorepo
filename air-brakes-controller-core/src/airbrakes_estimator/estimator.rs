use biquad::{
    Biquad as _, Coefficients, DirectForm2Transposed, Q_BUTTERWORTH_F32, ToHertz as _, Type,
};
use heapless::Deque;
use micromath::F32Ext;
use nalgebra::{UnitQuaternion, UnitVector3, Vector2, Vector3};

use crate::{
    airbrakes_estimator::{
        AirbrakesConfig, MAX_DT_S, Measurement, NOMINAL_DT, NOMINAL_SAMPLES_PER_S,
        dead_reckoner::DeadReckoner, vertical_kf::VerticalKF, welford::Welford,
    },
    utils::approximate_speed_of_sound,
};

const UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

// --- Pad bias calibration (Piece 1) ---------------------------------------
/// Each completed calibration window is this long (s of measured time).
const BIAS_WINDOW_S: f32 = 2.0;
/// How many window means we keep (32 x 2 s = about the last minute on pad).
const MAX_BIAS_WINDOWS: usize = 32;
/// A window whose mean is further than this (rad/s, per axis) from the
/// median of all windows is thrown out: its "bias" was really rail sway.
const BIAS_REJECT_RAD_S: f32 = 0.00262; // 0.15 deg/s
/// Bias uncertainty assumed when there were not enough windows to screen
/// (rocket powered on very shortly before launch).
const FALLBACK_BIAS_SPREAD_RAD_S: f32 = 0.00524; // 0.3 deg/s

// --- Stage 1 (thrust-vector alignment) ------------------------------------
const STAGE1_DURATION_S: f32 = 0.5;

// --- Lockout exit vote (Piece 3) ------------------------------------------
/// Exit threshold with margin below the real Mach 0.8 requirement.
const VOTE_MACH: f32 = 0.75;
/// 2 of 3 votes must hold continuously this long before the baro is
/// declared honest.
const VOTE_SUSTAIN_S: f32 = 1.0;
/// Vote 3: the (port-corrected) baro climb rate must match the
/// dead-reckoned vertical velocity within this (m/s).
const RATE_AGREE_M_S: f32 = 15.0;
/// The baro-rate vote measures the slope over roughly this span (s).
const RATE_WINDOW_S: f32 = 0.5;
/// Ring of recent port-corrected baro samples: covers RATE_WINDOW_S with
/// margin even at a 500 Hz feed.
const BARO_RING_CAP: usize = 512;
const BARO_RING_SPAN_S: f32 = 0.7;

// --- Birth of the vertical filter (Piece 4) -------------------------------
/// Initial vertical-velocity uncertainty at a vote-approved birth (the
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
/// Tilt is capped here for the horizontal-velocity output and the airspeed
/// used in the port correction (tan/1/cos blow up at 90 deg, where the
/// brakes have no authority left anyway).
const TILT_CAP_RAD: f32 = 1.396; // 80 deg
/// A sample counts as clipped when any accel axis is within 2% of the
/// LSM6DSM's +/-16 g full scale.
const ACCEL_CLIP_LIMIT: f32 = 0.98 * 16.0 * 9.81;

#[derive(Debug)]
enum State {
    /// Stable on pad: collect gyro-bias windows, watch for ignition with a
    /// 2 s rolling buffer that is rewound once ignition is detected.
    OnPad {
        imu_data_list: Deque<Measurement, { NOMINAL_SAMPLES_PER_S * 2 }>,
        x_acc_low_pass: DirectForm2Transposed<f32>,
        y_acc_low_pass: DirectForm2Transposed<f32>,
        z_acc_low_pass: DirectForm2Transposed<f32>,
        bias_windows: heapless::Vec<Vector3<f32>, MAX_BIAS_WINDOWS>,
        bias_window_welford: Welford<3>,
        bias_window_elapsed: f32,
    },

    /// First half second of powered flight: the thrust direction tells us
    /// how the avionics are mounted in the rocket.
    Stage1 {
        elapsed: f32,
        acc_welford: Welford<3>,
        pad_av_orientation: UnitQuaternion<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        bias_spread: f32,
        launch_pad_altitude_asl: f32,
        ignition_t_us: u64,
    },

    /// Boost and Mach lockout: inertial dead reckoning only, no filter.
    /// The baro is watched (for the vote) but never fused.
    DeadReckoning {
        q_av_to_rocket: UnitQuaternion<f32>,
        reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        bias_spread: f32,
        launch_pad_altitude_asl: f32,
        ignition_t_us: u64,
        /// (timestamp_us, port-corrected baro altitude)
        baro_ring: Deque<(u64, f32), BARO_RING_CAP>,
        vote_sustain: f32,
        last_votes: (bool, bool, bool),
        accel_clipped: u32,
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
        accel_clipped: u32,
    },

    Apogee {
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
        accel_clipped: u32,
    },
}

/// The Phase B v2 airbrakes estimator. See the module docs and
/// ESTIMATOR_REWORK_PLAN.md for the design and the flight evidence behind
/// it.
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
        let acc_low_pass_coeff = Coefficients::<f32>::from_params(
            Type::LowPass,
            (NOMINAL_SAMPLES_PER_S as f32).hz(),
            10f32.hz(),
            Q_BUTTERWORTH_F32,
        )
        .unwrap();
        Self {
            state: State::OnPad {
                imu_data_list: Deque::new(),
                x_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
                y_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
                z_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
                bias_windows: heapless::Vec::new(),
                bias_window_welford: Welford::new(),
                bias_window_elapsed: 0.0,
            },
            config,
            prev_timestamp_us: None,
        }
    }

    /// Feed one timestamped IMU+baro sample.
    ///
    /// `deployment_speed`: the slow deployment estimator's current speed
    /// estimate (m/s), if available — it is vote 2 of the lockout exit.
    /// Pass `None` if that estimator is not running; the exit then needs
    /// both remaining votes.
    pub fn update(&mut self, z: &Measurement, deployment_speed: Option<f32>) {
        let dt = match self.prev_timestamp_us {
            Some(prev) => {
                ((z.timestamp_us.saturating_sub(prev)) as f32 * 1e-6).clamp(0.0, MAX_DT_S)
            }
            None => NOMINAL_DT,
        };
        self.prev_timestamp_us = Some(z.timestamp_us);

        let acc = z.acceleration();
        let gyro = z.angular_velocity();

        match &mut self.state {
            State::OnPad {
                imu_data_list,
                x_acc_low_pass,
                y_acc_low_pass,
                z_acc_low_pass,
                bias_windows,
                bias_window_welford,
                bias_window_elapsed,
            } => {
                let acc_low_passed = [
                    x_acc_low_pass.run(acc[0]),
                    y_acc_low_pass.run(acc[1]),
                    z_acc_low_pass.run(acc[2]),
                ];

                if imu_data_list.is_full() {
                    imu_data_list.pop_front().unwrap();
                }
                imu_data_list.push_back(z.clone()).unwrap();

                // Long-window bias collector: finished 2 s windows are kept
                // as candidate bias measurements; the current (possibly
                // ignition-shaken) window is never used.
                bias_window_welford.update(&gyro);
                *bias_window_elapsed += dt;
                if *bias_window_elapsed >= BIAS_WINDOW_S {
                    if bias_windows.is_full() {
                        bias_windows.remove(0);
                    }
                    let _ = bias_windows.push(bias_window_welford.mean());
                    *bias_window_welford = Welford::new();
                    *bias_window_elapsed = 0.0;
                }

                if !imu_data_list.is_full() {
                    return;
                }
                let acc_magnitude_squared: f32 = acc_low_passed.iter().map(|a| a * a).sum();
                let threshold = self.config.ignition_detection_acc_threshold;
                if acc_magnitude_squared <= threshold * threshold {
                    return;
                }
                log_info!("ignition detected, rewinding pad buffer");

                // Rewind: the buffer's first half is the still rocket
                // (gravity direction, pad altitude, fallback bias); the
                // second half is ignition shake, dead-reckoned through.
                let half = imu_data_list.len() / 2;
                let mut acc_w = Welford::<3>::new();
                let mut gyro_w = Welford::<3>::new();
                let mut alt_sum = 0.0f32;
                for past in imu_data_list.iter().take(half) {
                    acc_w.update(&past.acceleration());
                    gyro_w.update(&past.angular_velocity());
                    alt_sum += past.altitude_asl();
                }
                let launch_pad_altitude_asl = alt_sum / half as f32;

                let (gyro_bias, bias_spread) = screen_bias(bias_windows, gyro_w.mean());
                log_info!(
                    "gyro bias: screened over {} windows, spread {} deg/s",
                    bias_windows.len(),
                    bias_spread.to_degrees()
                );

                let gravity_vector_av_frame: Vector3<f32> = acc_w.mean();
                let pad_av_orientation =
                    quaternion_from_start_and_end_vector(&UP, &gravity_vector_av_frame);
                let mut reckoner = DeadReckoner::new(pad_av_orientation);
                reckoner.position.z = launch_pad_altitude_asl;

                let mut prev_t: Option<u64> = None;
                for past in imu_data_list.iter() {
                    let past_dt = match prev_t {
                        Some(p) => {
                            ((past.timestamp_us.saturating_sub(p)) as f32 * 1e-6).clamp(0.0, MAX_DT_S)
                        }
                        None => 0.0,
                    };
                    prev_t = Some(past.timestamp_us);
                    if past_dt > 0.0 {
                        // orientation-only through the still half too is
                        // harmless (rates are ~bias there)
                        reckoner.update(
                            &past.acceleration(),
                            &(past.angular_velocity() - gyro_bias),
                            past_dt,
                        );
                    }
                }

                log_info!("to stage 1: {:?}", reckoner);
                self.state = State::Stage1 {
                    elapsed: 0.0,
                    acc_welford: Welford::new(),
                    pad_av_orientation,
                    reckoner,
                    gyro_bias,
                    bias_spread,
                    launch_pad_altitude_asl,
                    ignition_t_us: z.timestamp_us,
                };
            }

            State::Stage1 {
                elapsed,
                acc_welford,
                pad_av_orientation,
                reckoner,
                gyro_bias,
                bias_spread,
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
                    reckoner: reckoner.clone(),
                    gyro_bias: *gyro_bias,
                    bias_spread: *bias_spread,
                    launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    ignition_t_us: *ignition_t_us,
                    baro_ring: Deque::new(),
                    vote_sustain: 0.0,
                    last_votes: (false, false, false),
                    accel_clipped: 0,
                };
            }

            State::DeadReckoning {
                q_av_to_rocket,
                reckoner,
                gyro_bias,
                bias_spread,
                launch_pad_altitude_asl,
                ignition_t_us,
                baro_ring,
                vote_sustain,
                last_votes,
                accel_clipped,
            } => {
                if acc.abs().max() >= ACCEL_CLIP_LIMIT {
                    *accel_clipped += 1;
                }
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);

                // Port-corrected baro, using the dead-reckoned airspeed
                // (the only speed available before the filter is born).
                let corrected =
                    z.altitude_asl() - self.config.baro_port_coefficient * reckoner.velocity.magnitude_squared();
                if baro_ring.is_full() {
                    baro_ring.pop_front();
                }
                let _ = baro_ring.push_back((z.timestamp_us, corrected));
                while let Some(front) = baro_ring.front() {
                    if (z.timestamp_us.saturating_sub(front.0)) as f32 * 1e-6 > BARO_RING_SPAN_S {
                        baro_ring.pop_front();
                    } else {
                        break;
                    }
                }

                let t_since_ignition_s =
                    (z.timestamp_us.saturating_sub(*ignition_t_us)) as f32 * 1e-6;

                let (born, forced) = match &self.config.mach_lockout {
                    // Subsonic profile: the baro is honest as soon as we
                    // have enough of it buffered for a clean median.
                    None => (ring_span_s(baro_ring) >= 0.25, false),
                    Some(lockout) => {
                        let t_min_s = lockout.t_min_us as f32 * 1e-6;
                        let t_max_s = lockout.t_max_us as f32 * 1e-6;
                        if t_since_ignition_s >= t_max_s {
                            log_info!("lockout T_max reached, forced birth");
                            (true, true)
                        } else if t_since_ignition_s < t_min_s {
                            (false, false)
                        } else {
                            let sos = approximate_speed_of_sound(reckoner.position.z);

                            // Vote 1: dead-reckoned total speed, with an
                            // explicit margin for the pad-calibration bias
                            // uncertainty (tilt error grows as spread*t,
                            // and gravity misprojection turns that into
                            // velocity error at ~g*tilt_err*t).
                            let speed_margin =
                                9.81 * *bias_spread * t_since_ignition_s * t_since_ignition_s;
                            let v1 = reckoner.velocity.magnitude() + speed_margin
                                < VOTE_MACH * sos;

                            // Vote 2: the slow deployment filter agrees.
                            // It lags high during deceleration, so it errs
                            // late — the safe direction.
                            let v2 = deployment_speed
                                .map(|v| v.abs() < VOTE_MACH * sos)
                                .unwrap_or(false);

                            // Vote 3: the corrected baro's climb rate
                            // matches the dead-reckoned vertical velocity.
                            // A shock-corrupted baro has a wildly wrong
                            // slope; a merely drifted inertial altitude
                            // does not disturb the RATE.
                            let v3 = match baro_slope(baro_ring) {
                                Some(slope) => {
                                    (slope - reckoner.velocity.z).abs() < RATE_AGREE_M_S
                                }
                                None => false,
                            };

                            *last_votes = (v1, v2, v3);
                            let count = v1 as u8 + v2 as u8 + v3 as u8;
                            if count >= 2 {
                                *vote_sustain += dt;
                            } else {
                                *vote_sustain = 0.0;
                            }
                            (*vote_sustain >= VOTE_SUSTAIN_S, false)
                        }
                    }
                };

                if !born {
                    return;
                }

                // Birth ("born subsonic"): nothing from the garbage period
                // survives into the filter except these two numbers.
                let alt0 = match ring_median(baro_ring) {
                    Some(m) => m,
                    None => return, // no baro at all yet — wait
                };
                let vv0 = reckoner.velocity.z;
                let kf = VerticalKF::born(
                    alt0,
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
                    alt0,
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
                    accel_clipped: *accel_clipped,
                };
            }

            State::Tracking {
                q_av_to_rocket,
                reckoner,
                gyro_bias,
                launch_pad_altitude_asl,
                kf,
                apogee_sustain,
                baro_accept_age,
                last_raw_baro,
                baro_frozen_s,
                accel_clipped,
                ..
            } => {
                if acc.abs().max() >= ACCEL_CLIP_LIMIT {
                    *accel_clipped += 1;
                }
                // The dead reckoner keeps running for tilt (gyro-only
                // orientation); its velocity/position are no longer used.
                reckoner.update(&acc, &(gyro - *gyro_bias), dt);

                kf.predict(reckoner.acceleration.z, dt);

                // Port correction from the filter's own state: airspeed =
                // vv / cos(tilt), tilt capped so this stays sane at apogee
                // (and the correction correctly goes to zero with vv).
                let tilt = axis_tilt(q_av_to_rocket, reckoner).min(TILT_CAP_RAD);
                let airspeed = kf.vertical_velocity().abs() / tilt.cos();
                let corrected = z.altitude_asl()
                    - self.config.baro_port_coefficient * airspeed * airspeed;

                if kf.update(corrected, dt) {
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
                        accel_clipped: *accel_clipped,
                    };
                }
            }

            State::Apogee { .. } => {}
        }
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
                Some(Vector2::new((vv * tilt.tan()).abs(), vv))
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

    /// The three lockout-exit votes (dead-reckoned speed, deployment
    /// filter, baro rate) — for logging/telemetry. `None` outside the
    /// dead-reckoning phase.
    pub fn lockout_votes(&self) -> Option<(bool, bool, bool)> {
        match &self.state {
            State::DeadReckoning { last_votes, .. } => Some(*last_votes),
            _ => None,
        }
    }

    /// When and how the vertical filter was born: (timestamp_us, forced).
    /// `forced` means the T_max ceiling fired instead of the vote.
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

    /// How many accel samples hit the sensor's +/-16 g limit since
    /// ignition. Nonzero means the dead-reckoned velocity is degraded
    /// (telemetry should flag it).
    pub fn accel_clipped_samples(&self) -> u32 {
        match &self.state {
            State::DeadReckoning { accel_clipped, .. }
            | State::Tracking { accel_clipped, .. }
            | State::Apogee { accel_clipped, .. } => *accel_clipped,
            _ => 0,
        }
    }
}

/// Pick the gyro bias from the completed pad windows: take the per-axis
/// median of the window means, throw out windows that disagree with it
/// (their "bias" was rail sway, the measured failure mode), average the
/// rest. Returns (bias, spread) where spread is how much the surviving
/// windows still disagree — carried into the Mach-check margin.
fn screen_bias(
    windows: &heapless::Vec<Vector3<f32>, MAX_BIAS_WINDOWS>,
    fallback: Vector3<f32>,
) -> (Vector3<f32>, f32) {
    if windows.len() < 3 {
        return (fallback, FALLBACK_BIAS_SPREAD_RAD_S);
    }

    let mut median = Vector3::<f32>::zeros();
    for axis in 0..3 {
        let mut vals: heapless::Vec<f32, MAX_BIAS_WINDOWS> = heapless::Vec::new();
        for w in windows.iter() {
            let _ = vals.push(w[axis]);
        }
        vals.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        median[axis] = vals[vals.len() / 2];
    }

    let mut sum = Vector3::<f32>::zeros();
    let mut n = 0usize;
    for w in windows.iter() {
        let d = w - median;
        if d.abs().max() <= BIAS_REJECT_RAD_S {
            sum += w;
            n += 1;
        }
    }
    if n < 2 {
        // Everything disagrees with everything: heavy sway. Use the median
        // (robust) and be honest about the uncertainty.
        return (median, FALLBACK_BIAS_SPREAD_RAD_S);
    }
    let bias = sum / n as f32;

    let mut spread = 0.0f32;
    for w in windows.iter() {
        let d = w - median;
        if d.abs().max() <= BIAS_REJECT_RAD_S {
            spread = spread.max((w - bias).magnitude());
        }
    }
    (bias, spread)
}

fn ring_span_s(ring: &Deque<(u64, f32), BARO_RING_CAP>) -> f32 {
    match (ring.front(), ring.back()) {
        (Some(front), Some(back)) => (back.0.saturating_sub(front.0)) as f32 * 1e-6,
        _ => 0.0,
    }
}

/// Slope (m/s) of the corrected baro over the ring, oldest-to-newest.
/// `None` until the ring spans most of the rate window.
fn baro_slope(ring: &Deque<(u64, f32), BARO_RING_CAP>) -> Option<f32> {
    let (front, back) = (ring.front()?, ring.back()?);
    let span = (back.0.saturating_sub(front.0)) as f32 * 1e-6;
    if span < RATE_WINDOW_S * 0.9 {
        return None;
    }
    Some((back.1 - front.1) / span)
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
/// end vector
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
