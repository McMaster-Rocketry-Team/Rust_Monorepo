//! Baro-only flight state machine.
//!
//! Detects ignition, ascent, descent, and landing from barometric altitude alone
//! (2-state Kalman filter). Supports single (both pyros at apogee) and dual
//! (drogue at apogee, main at altitude) deployment via [`FlightProfile`].

mod altitude_kf;

#[cfg(test)]
mod tests;

pub use altitude_kf::BaroAltitudeKF;

use crate::utils::approximate_speed_of_sound;
use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;

/// Baro sample rate the estimator is designed for (matches IMU ODR).
pub const SAMPLES_PER_S: usize = 416;
pub const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

/// Vertical velocity above which (together with altitude rise) ignition is detected
const IGNITION_VELOCITY_THRESHOLD: f32 = 10.0; // m/s
/// Altitude rise above launch pad required for ignition detection
const IGNITION_ALTITUDE_RISE: f32 = 15.0; // m
/// Sustained vertical velocity below this value counts as descending
const DESCENT_VELOCITY_THRESHOLD: f32 = -2.0; // m/s
/// Low-passed vertical acceleration below this during ascent means the motor
/// burned out and the rocket is coasting (gravity + drag only)
const COAST_ACCEL_THRESHOLD: f32 = -4.0; // m/s^2
/// Time constant of the burnout-detection acceleration low-pass filter
const COAST_ACCEL_FILTER_TIME_CONSTANT: f32 = 0.3; // s
/// Consecutive samples of low-passed acceleration below the threshold before
/// coasting is latched. Reset-on-violation: this assumes a stall-free sensor
/// stream — the KF re-convergence burst after a dropped-sample gap reads as a
/// brief positive-acceleration spike that resets the counter (on the Void Lake
/// data, which had 1 Hz stalls, that deferred the latch by ~10 s).
const COAST_DETECTION_SAMPLES: usize = SAMPLES_PER_S; // 1 s
/// How long the rocket has to descend before the descent is acted upon
const DESCENT_DETECTION_SAMPLES: usize = SAMPLES_PER_S / 2; // 0.5 s
/// |KF vertical velocity| below this counts as standing still. Sized ~5 sigma
/// above the stationary velocity-estimate noise (std 0.37 m/s, peaks ~1.6 m/s
/// over minutes), so noise never resets the landed counter, while canopy
/// descent (>= ~4.5 m/s) keeps it pinned at zero.
const LANDED_VELOCITY_THRESHOLD: f32 = 2.0; // m/s
/// How long the rocket has to stand still before it is considered landed
const LANDED_DETECTION_SAMPLES: usize = SAMPLES_PER_S * 5; // 5 s
/// Time constant of the launch pad altitude low-pass filter
const PAD_ALTITUDE_FILTER_TIME_CONSTANT: f32 = 10.0; // s
/// KF speed, as a fraction of the local speed of sound, above which the baro
/// is locked out (see [`Stage::MachLockout`]). Transonic shock formation
/// starts near Mach 0.8; entering at 0.75 leaves margin. Entry may trust the
/// KF velocity because the subsonic static-port error only reads *fast* —
/// it makes entry early, never late.
const MACH_LOCKOUT_ENTER: f32 = 0.75;

/// Flight deployment profile: single (both at apogee) or dual (drogue then main).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub enum FlightProfile {
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

impl FlightProfile {
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RocketState {
    OnPad,
    Ascent {
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
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

#[derive(Debug, Clone)]
enum Stage {
    OnPad {
        /// low-passed launch pad altitude, tracks slow baro drift
        pad_altitude_asl: f32,
    },
    Ascent {
        launch_pad_altitude_asl: f32,
        descending_samples: usize,
        /// KF velocity at the previous sample, for the acceleration estimate
        prev_velocity: f32,
        /// low-passed d(velocity)/dt, negative once the motor burns out
        accel_lp: f32,
        /// consecutive samples with `accel_lp` below the coasting threshold
        coast_samples: usize,
        /// latched burnout: motors don't relight, so coasting never clears
        coasting: bool,
        /// the Mach lockout is one-shot: the speed profile of an ascent is
        /// unimodal, so there is exactly one contiguous window above the
        /// entry threshold and a post-exit glitch must not re-lock
        lockout_done: bool,
    },
    /// Baro readings are garbage around and above Mach 1 (shocks over the
    /// static port), and with baro as the only sensor there is no trustworthy
    /// signal to exit on — so the KF is frozen (no predict, no update) for a
    /// sim-derived duration, then re-seeded from fresh measurements. While
    /// frozen, no state transition can trigger: velocity is stuck positive so
    /// descent never accrues, and d(velocity)/dt reads zero so coasting never
    /// latches.
    MachLockout {
        launch_pad_altitude_asl: f32,
        samples_left: usize,
        /// carried through so a burnout latched before entry isn't lost
        coasting: bool,
    },
    DrogueDelay {
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    DrogueDeployed {
        launch_pad_altitude_asl: f32,
    },
    MainDelay {
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    MainDeployed {
        launch_pad_altitude_asl: f32,
        /// consecutive samples with |velocity| below the landed threshold
        still_samples: usize,
    },
    Landed {
        launch_pad_altitude_asl: f32,
    },
    FailedToReachMinApogee,
}

/// Baro-only state estimator + flight state machine.
///
/// Feed it baro altitude ASL at [`SAMPLES_PER_S`] via [`Self::update`].
#[derive(Debug, Clone)]
pub struct RocketStateEstimator {
    profile: FlightProfile,
    /// How long the baro is ignored after the KF speed first crosses
    /// [`MACH_LOCKOUT_ENTER`] x speed of sound. `None` disables the lockout
    /// (subsonic rockets).
    mach_lockout_duration_us: Option<u32>,
    kf: Option<BaroAltitudeKF>,
    stage: Stage,
}

fn us_to_ticks(us: u32) -> usize {
    // Round up so a non-zero delay always waits at least one sample.
    let ticks = (us as u64 * SAMPLES_PER_S as u64).div_ceil(1_000_000);
    ticks as usize
}

impl RocketStateEstimator {
    /// `mach_lockout_duration_us`: for flights exceeding ~Mach 0.75, how long
    /// the baro is ignored after the KF speed first crosses the entry
    /// threshold. Take (time above Mach 0.75) from the flight sim with ~1.4x
    /// margin; it must still end well (>5 s) before apogee. `None` disables
    /// the lockout — use that for subsonic rockets.
    pub fn new(profile: FlightProfile, mach_lockout_duration_us: Option<u32>) -> Self {
        Self {
            profile,
            mach_lockout_duration_us,
            kf: None,
            stage: Stage::OnPad {
                pad_altitude_asl: 0.0,
            },
        }
    }

    /// Process one baro altitude ASL sample (m).
    /// Returns `Some(pyro)` when a pyro channel should be fired.
    pub fn update(&mut self, baro_altitude_asl: f32) -> Option<PyroSelect> {
        let kf = match &mut self.kf {
            Some(kf) => {
                // During Mach lockout the KF is frozen: predicting on the
                // constant-velocity model while decelerating at 2-3 g would
                // accumulate km of error, and the measurements are garbage.
                if !matches!(self.stage, Stage::MachLockout { .. }) {
                    kf.predict();
                    kf.update(baro_altitude_asl);
                }
                kf
            }
            None => {
                self.stage = Stage::OnPad {
                    pad_altitude_asl: baro_altitude_asl,
                };
                self.kf.insert(BaroAltitudeKF::new(baro_altitude_asl))
            }
        };
        let altitude = kf.altitude();
        let velocity = kf.vertical_velocity();

        let mut deploy_pyro = None;

        match &mut self.stage {
            Stage::OnPad { pad_altitude_asl } => {
                let alpha = DT / PAD_ALTITUDE_FILTER_TIME_CONSTANT;
                *pad_altitude_asl += alpha * (altitude - *pad_altitude_asl);

                if velocity > IGNITION_VELOCITY_THRESHOLD
                    && altitude - *pad_altitude_asl > IGNITION_ALTITUDE_RISE
                {
                    log_info!(
                        "ignition detected: v={}m/s, pad asl={}m",
                        velocity,
                        *pad_altitude_asl
                    );
                    self.stage = Stage::Ascent {
                        launch_pad_altitude_asl: *pad_altitude_asl,
                        descending_samples: 0,
                        prev_velocity: velocity,
                        accel_lp: 0.0,
                        coast_samples: 0,
                        coasting: false,
                        lockout_done: false,
                    };
                }
            }
            Stage::Ascent {
                launch_pad_altitude_asl,
                descending_samples,
                prev_velocity,
                accel_lp,
                coast_samples,
                coasting,
                lockout_done,
            } => {
                let accel = (velocity - *prev_velocity) / DT;
                *prev_velocity = velocity;
                *accel_lp += (DT / COAST_ACCEL_FILTER_TIME_CONSTANT) * (accel - *accel_lp);
                if !*coasting {
                    if *accel_lp < COAST_ACCEL_THRESHOLD {
                        *coast_samples += 1;
                        if *coast_samples >= COAST_DETECTION_SAMPLES {
                            log_info!(
                                "burnout detected: accel={}m/s^2, v={}m/s",
                                *accel_lp,
                                velocity
                            );
                            *coasting = true;
                        }
                    } else {
                        *coast_samples = 0;
                    }
                }

                if !*lockout_done
                    && let Some(duration_us) = self.mach_lockout_duration_us
                    && velocity.abs() > MACH_LOCKOUT_ENTER * approximate_speed_of_sound(altitude)
                {
                    log_info!("mach lockout entered: v={}m/s", velocity);
                    self.stage = Stage::MachLockout {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: us_to_ticks(duration_us),
                        coasting: *coasting,
                    };
                } else {
                    if velocity < DESCENT_VELOCITY_THRESHOLD {
                        *descending_samples += 1;
                    } else {
                        *descending_samples = 0;
                    }

                    if *descending_samples >= DESCENT_DETECTION_SAMPLES {
                        let altitude_agl = altitude - *launch_pad_altitude_asl;
                        let min_agl = self.profile.minimum_deployment_agl();
                        if altitude_agl < min_agl {
                            log_info!(
                                "failed to reach min apogee: min={}, current={}",
                                min_agl,
                                altitude_agl
                            );
                            self.stage = Stage::FailedToReachMinApogee;
                        } else {
                            log_info!("descent detected: agl={}m", altitude_agl);
                            self.stage = Stage::DrogueDelay {
                                launch_pad_altitude_asl: *launch_pad_altitude_asl,
                                samples_left: us_to_ticks(self.profile.drogue_delay_us()),
                            };
                        }
                    }
                }
            }
            Stage::MachLockout {
                launch_pad_altitude_asl,
                samples_left,
                coasting,
            } => {
                *samples_left = samples_left.saturating_sub(1);
                if *samples_left == 0 {
                    log_info!("mach lockout over, reseeding KF at {}m", baro_altitude_asl);
                    if let Some(kf) = &mut self.kf {
                        kf.reseed(baro_altitude_asl);
                    }
                    self.stage = Stage::Ascent {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        descending_samples: 0,
                        prev_velocity: 0.0,
                        accel_lp: 0.0,
                        coast_samples: 0,
                        coasting: *coasting,
                        lockout_done: true,
                    };
                }
            }
            Stage::DrogueDelay {
                launch_pad_altitude_asl,
                samples_left,
            } => {
                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroDrogue);
                    let pad = *launch_pad_altitude_asl;
                    if self.profile.is_single() {
                        // Single: main follows drogue with no extra delay
                        // (main_delay_us() == 0), so it fires on the next sample.
                        self.stage = Stage::MainDelay {
                            launch_pad_altitude_asl: pad,
                            samples_left: us_to_ticks(self.profile.main_delay_us()),
                        };
                    } else {
                        self.stage = Stage::DrogueDeployed {
                            launch_pad_altitude_asl: pad,
                        };
                    }
                } else {
                    *samples_left -= 1;
                }
            }
            Stage::DrogueDeployed {
                launch_pad_altitude_asl,
            } => {
                // Dual only: wait for main altitude.
                if let Some(main_agl) = self.profile.main_chute_altitude_agl()
                    && altitude < main_agl + *launch_pad_altitude_asl
                {
                    self.stage = Stage::MainDelay {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: us_to_ticks(self.profile.main_delay_us()),
                    };
                }
            }
            Stage::MainDelay {
                launch_pad_altitude_asl,
                samples_left,
            } => {
                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroMain);
                    self.stage = Stage::MainDeployed {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        still_samples: 0,
                    };
                } else {
                    *samples_left -= 1;
                }
            }
            Stage::MainDeployed {
                launch_pad_altitude_asl,
                still_samples,
            } => {
                if velocity.abs() < LANDED_VELOCITY_THRESHOLD {
                    *still_samples += 1;
                } else {
                    *still_samples = 0;
                }

                if *still_samples >= LANDED_DETECTION_SAMPLES {
                    log_info!("landed");
                    self.stage = Stage::Landed {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Stage::Landed { .. } | Stage::FailedToReachMinApogee => {}
        }

        deploy_pyro
    }

    pub fn state(&self) -> RocketState {
        let (altitude, velocity) = match &self.kf {
            Some(kf) => (kf.altitude(), kf.vertical_velocity()),
            None => (0.0, 0.0),
        };

        match &self.stage {
            Stage::OnPad { .. } => RocketState::OnPad,
            Stage::Ascent {
                launch_pad_altitude_asl,
                ..
            } => RocketState::Ascent {
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            // Reported as Ascent with the frozen (stale) KF values; check
            // `in_mach_lockout()` before acting on them.
            Stage::MachLockout {
                launch_pad_altitude_asl,
                ..
            } => RocketState::Ascent {
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::DrogueDelay {
                launch_pad_altitude_asl,
                ..
            } => RocketState::DrogueChute {
                deployed: false,
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::DrogueDeployed {
                launch_pad_altitude_asl,
            } => RocketState::DrogueChute {
                deployed: true,
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::MainDelay {
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: false,
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::MainDeployed {
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: true,
                vertical_velocity: velocity,
                altitude_asl: altitude,
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Stage::Landed { .. } => RocketState::Landed,
            Stage::FailedToReachMinApogee => RocketState::FailedToReachMinApogee,
        }
    }

    /// True during ascent once the motor has burned out (the low-passed vertical
    /// acceleration turned negative), i.e. the rocket is coasting to apogee.
    pub fn is_coasting(&self) -> bool {
        matches!(
            self.stage,
            Stage::Ascent { coasting: true, .. } | Stage::MachLockout { coasting: true, .. }
        )
    }

    /// True while the baro is locked out around/above Mach 1: the KF is
    /// frozen and everything derived from it (state, altitude, velocity) is
    /// stale — nothing downstream should act on those values.
    pub fn in_mach_lockout(&self) -> bool {
        matches!(self.stage, Stage::MachLockout { .. })
    }

    pub fn altitude_asl(&self) -> f32 {
        self.kf.as_ref().map(|kf| kf.altitude()).unwrap_or(0.0)
    }

    pub fn vertical_velocity(&self) -> f32 {
        self.kf
            .as_ref()
            .map(|kf| kf.vertical_velocity())
            .unwrap_or(0.0)
    }

    pub fn launch_pad_altitude_asl(&self) -> f32 {
        match &self.stage {
            Stage::OnPad { pad_altitude_asl } => *pad_altitude_asl,
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
            } => *launch_pad_altitude_asl,
            Stage::FailedToReachMinApogee => 0.0,
        }
    }

    pub fn altitude_agl(&self) -> f32 {
        self.altitude_asl() - self.launch_pad_altitude_asl()
    }
}
