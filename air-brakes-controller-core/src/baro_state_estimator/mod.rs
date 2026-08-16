//! Baro-only *deployment* state machine.
//!
//! Detects ignition, apogee, and landing from barometric altitude alone via a
//! deliberately slow (~1 s bandwidth) 2-state Kalman filter whose output is
//! trusted outright — the COTS-altimeter shape: innovation gate as bus input
//! validation, a timed Mach lockout started at ignition detection, apogee by
//! peak-drop on the filtered altitude, coasting by burn timer, and
//! "condition holds for N samples" persistence on every transition. Accuracy
//! is explicitly not a goal (boost lag is hundreds of metres); the airbrakes
//! use a separate fast estimator. Supports single (both pyros at apogee) and
//! dual (drogue at apogee, main at altitude) deployment via [`FlightProfile`].

mod altitude_kf;

#[cfg(test)]
mod tests;

pub use altitude_kf::BaroAltitudeKF;

use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;

use crate::baro_gate::BaroGateOutcome;

/// Baro sample rate the estimator is designed for (matches IMU ODR).
pub const SAMPLES_PER_S: usize = 416;
pub const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

/// Vertical velocity above which (together with altitude rise) ignition is detected
const IGNITION_VELOCITY_THRESHOLD: f32 = 10.0; // m/s
/// Altitude rise above launch pad required for ignition detection
const IGNITION_ALTITUDE_RISE: f32 = 15.0; // m
/// Apogee detection: filtered altitude this far below its running maximum
/// counts as descending. Must exceed the worst transient dip a gate-leaking
/// blast can put on the slow filter (~30 m from a 25-sample 500 m offset,
/// which then decays in ~1 s — too short for the persistence window below).
const APOGEE_DROP_M: f32 = 30.0; // m
/// How long the altitude has to stay below (peak - APOGEE_DROP_M) before
/// descent is acted upon
const APOGEE_DROP_SAMPLES: usize = SAMPLES_PER_S / 2; // 0.5 s
/// |KF vertical velocity| below this counts as standing still. The slow
/// filter's stationary velocity noise is ~0.012 m/s std (peaks ~0.05 m/s), so
/// this is sized by canopy-swing and post-touchdown-drift rejection, not
/// noise; descent under main (>= ~4.5 m/s) keeps the counter pinned at zero.
const LANDED_VELOCITY_THRESHOLD: f32 = 2.0; // m/s
/// How long the rocket has to stand still before it is considered landed
const LANDED_DETECTION_SAMPLES: usize = SAMPLES_PER_S * 5; // 5 s
/// Time constant of the launch pad altitude low-pass filter
const PAD_ALTITUDE_FILTER_TIME_CONSTANT: f32 = 10.0; // s

/// Per-rocket flight configuration for the deployment estimator.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct FlightProfile {
    /// Baro Mach lockout: starting at THIS estimator's baro ignition
    /// detection, the KF is frozen (no predict, no update) for this long,
    /// then re-seeded — supersonic static-port readings are garbage.
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
    /// Unrelated to
    /// [`MachLockoutConfig`](crate::airbrakes_estimator::MachLockoutConfig)
    /// despite the similar name: that one bounds the airbrakes drag check and
    /// runs from a different sensor's ignition detection. Equal values in a
    /// config are coincidence, not a link.
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
/// stale. A caller who needs the raw (possibly frozen) KF numbers for
/// logging must ask for them explicitly via
/// [`RocketStateEstimator::kf_altitude_asl`] /
/// [`RocketStateEstimator::kf_vertical_velocity`].
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

#[derive(Debug, Clone)]
enum Stage {
    OnPad {
        /// low-passed launch pad altitude, tracks slow baro drift
        pad_altitude_asl: f32,
    },
    Ascent {
        launch_pad_altitude_asl: f32,
        /// running maximum of the filtered altitude; apogee is detected when
        /// the altitude drops [`APOGEE_DROP_M`] below it
        peak_altitude_asl: f32,
        /// consecutive samples with altitude below (peak - APOGEE_DROP_M)
        below_peak_samples: usize,
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
        samples_left: usize,
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
    FailedToReachMinApogee {
        launch_pad_altitude_asl: f32,
    },
}

/// Baro-only state estimator + flight state machine.
///
/// Feed it baro altitude ASL at [`SAMPLES_PER_S`] via [`Self::update`].
#[derive(Debug, Clone)]
pub struct RocketStateEstimator {
    profile: FlightProfile,
    kf: Option<BaroAltitudeKF>,
    stage: Stage,
    /// What the innovation gate did with the sample this estimator last
    /// processed. `Accepted` while the KF is frozen in Mach lockout —
    /// nothing is fused there, so nothing is rejected either.
    last_baro_gate: BaroGateOutcome,
}

fn us_to_ticks(us: u32) -> usize {
    // Round up so a non-zero delay always waits at least one sample.
    let ticks = (us as u64 * SAMPLES_PER_S as u64).div_ceil(1_000_000);
    ticks as usize
}

impl RocketStateEstimator {
    pub fn new(profile: FlightProfile) -> Self {
        Self {
            profile,
            kf: None,
            stage: Stage::OnPad {
                pad_altitude_asl: 0.0,
            },
            last_baro_gate: BaroGateOutcome::Accepted,
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
                    self.last_baro_gate = kf.update(baro_altitude_asl);
                } else {
                    self.last_baro_gate = BaroGateOutcome::Accepted;
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
        let altitude_asl = kf.altitude_asl();
        let velocity = kf.vertical_velocity();

        let mut deploy_pyro = None;

        match &mut self.stage {
            Stage::OnPad { pad_altitude_asl } => {
                let alpha = DT / PAD_ALTITUDE_FILTER_TIME_CONSTANT;
                *pad_altitude_asl += alpha * (altitude_asl - *pad_altitude_asl);

                if velocity > IGNITION_VELOCITY_THRESHOLD
                    && altitude_asl - *pad_altitude_asl > IGNITION_ALTITUDE_RISE
                {
                    log_info!(
                        "ignition detected: v={}m/s, pad asl={}m",
                        velocity,
                        *pad_altitude_asl
                    );
                    let pad = *pad_altitude_asl;
                    self.stage = match self.profile.mach_lockout_duration_us {
                        Some(duration_us) => {
                            log_info!("mach lockout for {}us", duration_us);
                            Stage::MachLockout {
                                launch_pad_altitude_asl: pad,
                                samples_left: us_to_ticks(duration_us),
                            }
                        }
                        None => Stage::Ascent {
                            launch_pad_altitude_asl: pad,
                            peak_altitude_asl: altitude_asl,
                            below_peak_samples: 0,
                        },
                    };
                }
            }
            Stage::Ascent {
                launch_pad_altitude_asl,
                peak_altitude_asl,
                below_peak_samples,
            } => {
                if altitude_asl > *peak_altitude_asl {
                    *peak_altitude_asl = altitude_asl;
                }

                if *peak_altitude_asl - altitude_asl > APOGEE_DROP_M {
                    *below_peak_samples += 1;
                } else {
                    *below_peak_samples = 0;
                }

                if *below_peak_samples >= APOGEE_DROP_SAMPLES {
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
                            samples_left: us_to_ticks(self.profile.deployment.drogue_delay_us()),
                        };
                    }
                }
            }
            Stage::MachLockout {
                launch_pad_altitude_asl,
                samples_left,
            } => {
                *samples_left = samples_left.saturating_sub(1);
                if *samples_left == 0 {
                    log_info!("mach lockout over, reseeding KF at {}m", baro_altitude_asl);
                    if let Some(kf) = &mut self.kf {
                        kf.reseed(baro_altitude_asl);
                    }
                    self.stage = Stage::Ascent {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        peak_altitude_asl: baro_altitude_asl,
                        below_peak_samples: 0,
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
                    if self.profile.deployment.is_single() {
                        // Single: main follows drogue with no extra delay
                        // (main_delay_us() == 0), so it fires on the next sample.
                        self.stage = Stage::MainDelay {
                            launch_pad_altitude_asl: pad,
                            samples_left: us_to_ticks(self.profile.deployment.main_delay_us()),
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
                if let Some(main_agl) = self.profile.deployment.main_chute_altitude_agl()
                    && altitude_asl < main_agl + *launch_pad_altitude_asl
                {
                    self.stage = Stage::MainDelay {
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: us_to_ticks(self.profile.deployment.main_delay_us()),
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
            Stage::Landed { .. } | Stage::FailedToReachMinApogee { .. } => {}
        }

        deploy_pyro
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

    /// Raw KF altitude ASL (m) for the fast flight-log record ONLY. During
    /// the Mach lockout the KF is frozen, so this value may be stale by tens
    /// of seconds — for logging/diagnostics, never for control decisions.
    /// The honest interface is [`Self::state`], whose
    /// [`RocketState::MachLockout`] variant carries no altitude at all.
    /// What the innovation gate did with the sample this estimator last
    /// processed. Read it immediately after [`Self::update`]: it describes
    /// that one sample and is overwritten by the next.
    pub fn baro_gate(&self) -> BaroGateOutcome {
        self.last_baro_gate
    }

    pub fn kf_altitude_asl(&self) -> f32 {
        self.kf.as_ref().map(|kf| kf.altitude_asl()).unwrap_or(0.0)
    }

    /// Raw KF vertical velocity (m/s) for the fast flight-log record ONLY.
    /// Like [`Self::kf_altitude_asl`], may be frozen/stale during the Mach
    /// lockout — for logging/diagnostics, never for control decisions. The
    /// honest interface is [`Self::state`].
    pub fn kf_vertical_velocity(&self) -> f32 {
        self.kf
            .as_ref()
            .map(|kf| kf.vertical_velocity())
            .unwrap_or(0.0)
    }

    /// Launch pad altitude ASL (m), available in every stage: while on the
    /// pad this is the current low-passed pad estimate (tracking slow baro
    /// drift); from ignition detection onward it is the value latched at
    /// detection.
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
            }
            | Stage::FailedToReachMinApogee {
                launch_pad_altitude_asl,
            } => *launch_pad_altitude_asl,
        }
    }
}
