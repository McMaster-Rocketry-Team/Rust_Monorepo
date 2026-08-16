//! [`FlightEstimators`] — both flight estimators and the policy connecting
//! them, in one struct, so firmware holds ONE thing behind ONE mutex.
//!
//! Composition philosophy: the two estimators are fully independent — zero
//! shared state, different sensors, different clocks, different consumers.
//!
//! * The **deployment** half ([`RocketStateEstimator`]) is baro-only and
//!   sample-clocked: it assumes the real ~416 Hz stream (which armed mode
//!   provides) and its output is trusted outright — it fires the pyros.
//!   It never reads anything from the airbrakes half.
//! * The **airbrakes** half ([`AirbrakesEstimator`]) is IMU+baro and
//!   wall-clock: every integration step uses the measured dt carried in
//!   the [`Measurement`] timestamp, so sensor stalls and skipped samples
//!   are integrated honestly. It is accuracy-only: its output feeds the
//!   MPC, never the pyros.
//!
//! The only data that crosses between the two does so **by value, inside
//! [`FlightEstimators::update`], once per sample**: the deployment
//! estimator's speed feeds vote V2 of the airbrakes Mach-lockout exit
//! (abstaining while the slow filter is itself locked out), and the
//! deployment burn timer's coasting flag is one clause of the airbrakes
//! open gate. There are deliberately no `&mut` component accessors, so no
//! additional coupling can be introduced from outside this module.
//!
//! Failure direction of the gate: every clause of
//! [`FlightEstimators::airbrakes_mpc_states`] fails toward `None` — if
//! anything is missing, stale, or out of range, the brakes stay shut.
//! Recovery (the pyro path) does not depend on the airbrakes half at all.

use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use nalgebra::{Vector2, Vector3};

use crate::airbrakes_estimator::{AirbrakesConfig, AirbrakesEstimator, Measurement};
use crate::baro_state_estimator::{FlightProfile, RocketState, RocketStateEstimator};
use crate::utils::approximate_speed_of_sound;

/// Ceiling on the vertical velocity at which the gate will hand out MPC
/// states, as a fraction of the local speed of sound: the airbrakes may
/// only open below Mach 0.85 *per the airbrakes filter's own estimate*.
///
/// This is the deliberately-conservative slow-gate ceiling, and it is
/// distinct from the lockout-exit vote's Mach 0.8 threshold on purpose:
/// the vote decides *when the baro is honest again* and sits at the
/// actual "safe to open" requirement; this ceiling is an independent
/// last-layer sanity bound on the born filter's state. It sits above the
/// vote threshold so it never fights a healthy vote — it only bites if
/// the filter was born reporting near-sonic speed, in which case the
/// brakes stay shut until the estimate decays below it.
pub const MAX_OPEN_MACH: f32 = 0.85;

/// One IMU sample in the avionics frame, SI units at the API boundary:
/// specific force in m/s^2, angular velocity in rad/s. Firmware converts
/// units at the edge (e.g. deg/s -> rad/s) before constructing this.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct ImuSample {
    /// Accelerometer specific force (m/s^2).
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub acc: Vector3<f32>,
    /// Angular velocity (rad/s).
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub gyro: Vector3<f32>,
}

/// The MPC's input state, handed out by
/// [`FlightEstimators::airbrakes_mpc_states`] exactly when the airbrakes
/// are permitted to open. Permission and state availability are one
/// `Option` — "permitted but no state" cannot be expressed.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct AirbrakesMPCStates {
    /// Altitude ASL (m), from the airbrakes filter.
    pub altitude_asl: f32,
    /// Velocity `[horizontal, vertical]` (m/s), from the airbrakes filter.
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub velocity: Vector2<f32>,
}

/// The two flight estimators plus the policy connecting them. See the
/// module docs for the composition philosophy.
#[derive(Debug)]
pub struct FlightEstimators {
    deployment: RocketStateEstimator,
    airbrakes: AirbrakesEstimator,
}

impl FlightEstimators {
    pub fn new(profile: FlightProfile, config: AirbrakesConfig) -> Self {
        Self {
            deployment: RocketStateEstimator::new(profile),
            airbrakes: AirbrakesEstimator::new(config),
        }
    }

    /// The ONLY mutating function — call once per ~416 Hz sample.
    ///
    /// Baro is always present: the deployment estimator is sample-clocked
    /// and must see every sample. IMU is optional: when `imu` is `None`
    /// the airbrakes estimator is skipped entirely for this sample — its
    /// measured-dt integration bridges the gap at the next IMU sample.
    ///
    /// Returns the deployment estimator's pyro command passed through
    /// UNTOUCHED — this struct adds no policy to recovery.
    pub fn update(
        &mut self,
        timestamp_us: u64,
        imu: Option<&ImuSample>,
        baro_altitude_asl: f32,
    ) -> Option<PyroSelect> {
        // (a) Deployment first: baro-only, sample-clocked, trusted
        // outright. Its pyro command is returned as-is at the bottom.
        let pyro = self.deployment.update(baro_altitude_asl);

        // (b) The V2 vote feed, derived by value from the state the
        // deployment estimator just produced: `Some(vertical_velocity)`
        // exactly when the state variant carries a live velocity, `None`
        // (honest abstention) when it does not.
        //
        // The `MachLockout` arm is the load-bearing one: while the slow
        // filter is locked out its KF is frozen, and the stale (low)
        // velocity would make V2 spuriously vote "subsonic" — degrading
        // the airbrakes 2-of-3 lockout-exit vote to "V1 or V3". Abstaining
        // instead forces the vote to require V1 AND V3 in that window.
        // The variant carrying no velocity fields makes the stale read
        // unrepresentable, not merely avoided.
        let deployment_speed = match self.deployment.state() {
            RocketState::Ascent {
                vertical_velocity, ..
            }
            | RocketState::DrogueChute {
                vertical_velocity, ..
            }
            | RocketState::MainChute {
                vertical_velocity, ..
            } => Some(vertical_velocity),
            RocketState::OnPad
            | RocketState::MachLockout { .. }
            | RocketState::Landed
            | RocketState::FailedToReachMinApogee => None,
        };

        // (c) Airbrakes, only when this sample actually carries IMU data.
        if let Some(imu) = imu {
            let z = Measurement::new(timestamp_us, &imu.acc, &imu.gyro, baro_altitude_asl);
            self.airbrakes.update(&z, deployment_speed);
        }

        pyro
    }

    /// `Some` exactly when the airbrakes are permitted to open, carrying
    /// the MPC's input state. Permission and state are one `Option`, so
    /// "permitted but no state" cannot be expressed — the MPC's run/stop
    /// condition and its state source are the same value.
    ///
    /// The gate, in order:
    /// * coasting — the slow filter's burn timer, the one "never under
    ///   thrust" input, kept on the most trusted source;
    /// * the airbrakes filter is alive — baro trusted, pre-apogee, and
    ///   its altitude and velocity exist;
    /// * ascending (vertical velocity > 0);
    /// * vertical velocity at most [`MAX_OPEN_MACH`] of the local speed
    ///   of sound at the filter's own altitude.
    ///
    /// Every clause after coasting is evaluated on the airbrakes filter's
    /// OWN state — never the slow filter's, which may be frozen (Mach
    /// lockout) or lagging hundreds of metres during coast. Any clause
    /// failing yields `None`: the brakes stay shut.
    pub fn airbrakes_mpc_states(&self) -> Option<AirbrakesMPCStates> {
        if !self.deployment.is_coasting() {
            return None;
        }

        if !self.airbrakes.baro_trusted() || self.airbrakes.is_apogee() {
            return None;
        }
        let altitude_asl = self.airbrakes.altitude_asl()?;
        let velocity = self.airbrakes.velocity()?;

        let vertical_velocity = velocity.y;
        if vertical_velocity <= 0.0 {
            return None;
        }
        if vertical_velocity > MAX_OPEN_MACH * approximate_speed_of_sound(altitude_asl) {
            return None;
        }

        Some(AirbrakesMPCStates {
            altitude_asl,
            velocity,
        })
    }

    /// The deployment estimator's rocket state — the honest variant set,
    /// including [`RocketState::MachLockout`].
    pub fn state(&self) -> RocketState {
        self.deployment.state()
    }

    /// Read-only access to the deployment estimator (`is_coasting`,
    /// `launch_pad_altitude_asl`, telemetry assembly, ...).
    ///
    /// Deliberately no `&mut` twin: cross-estimator data flows by value,
    /// inside [`Self::update`], once per sample — the API makes any other
    /// coupling between the two estimators impossible to write.
    pub fn deployment_estimator(&self) -> &RocketStateEstimator {
        &self.deployment
    }

    /// Read-only access to the airbrakes estimator (votes, birth, tilt,
    /// fast-record flag assembly, ...). See [`Self::deployment_estimator`]
    /// for why there is no `&mut` twin.
    pub fn airbrakes_estimator(&self) -> &AirbrakesEstimator {
        &self.airbrakes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::baro_state_estimator::{DeploymentProfile, DT, SAMPLES_PER_S};
    use nalgebra::Vector3;

    /// Nominal 416 Hz sample spacing in microseconds.
    const SAMPLE_DT_US: u64 = 2404;

    fn test_profile() -> FlightProfile {
        FlightProfile {
            mach_lockout_duration_us: None,
            max_burn_time_us: 4_000_000,
            deployment: DeploymentProfile::Single {
                minimum_deployment_altitude_agl: 300.0,
                delay_us: 0,
            },
        }
    }

    fn test_config() -> AirbrakesConfig {
        AirbrakesConfig {
            ignition_detection_acc_threshold: 4.0 * 9.81,
            mach_lockout: None,
            baro_port_coefficient: 0.0,
        }
    }

    /// On the pad — before ignition, before the burn timer — the gate
    /// must hand out nothing, even with a healthy IMU feed coming in.
    #[test]
    fn no_mpc_states_before_coasting() {
        let mut est = FlightEstimators::new(test_profile(), test_config());
        let imu = ImuSample {
            acc: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::zeros(),
        };

        let mut t_us = 0u64;
        for _ in 0..(5 * SAMPLES_PER_S) {
            let pyro = est.update(t_us, Some(&imu), 200.0);
            assert!(pyro.is_none());
            assert!(est.airbrakes_mpc_states().is_none());
            t_us += SAMPLE_DT_US;
        }
        assert!(matches!(est.state(), RocketState::OnPad));
    }

    /// The composed update must return exactly what a bare deployment
    /// estimator returns on the same baro stream: pyro commands pass
    /// through untouched, none added, none dropped, none delayed.
    #[test]
    fn pyro_command_passes_through_unchanged() {
        let mut bare = RocketStateEstimator::new(test_profile());
        let mut composed = FlightEstimators::new(test_profile(), test_config());

        // Clean point-mass trajectory: 5 s pad hold, 3 s burn at
        // 80 m/s^2, ballistic coast over apogee, -25 m/s terminal
        // descent, 8 s on the ground (landed detection needs 5 s still).
        let pad = 200.0f32;
        let mut samples: Vec<f32> = Vec::new();
        samples.extend(core::iter::repeat(pad).take(5 * SAMPLES_PER_S));
        let mut altitude = pad;
        let mut velocity = 0.0f32;
        let mut t = 0.0f32;
        loop {
            let acceleration = if t < 3.0 { 80.0 } else { -9.81 };
            velocity = (velocity + acceleration * DT).max(-25.0);
            altitude += velocity * DT;
            t += DT;
            if altitude <= pad {
                break;
            }
            samples.push(altitude);
        }
        samples.extend(core::iter::repeat(pad).take(8 * SAMPLES_PER_S));

        // No IMU: the airbrakes half is skipped, the deployment half
        // (the one under test here) sees every sample either way.
        let mut fires = Vec::new();
        for (i, &alt) in samples.iter().enumerate() {
            let expected = bare.update(alt);
            let got = composed.update(i as u64 * SAMPLE_DT_US, None, alt);
            assert_eq!(expected, got, "pyro mismatch at sample {i}");
            if let Some(pyro) = got {
                fires.push(pyro);
            }
        }

        // Single deployment: drogue at apogee, main on the next sample.
        assert_eq!(fires, vec![PyroSelect::PyroDrogue, PyroSelect::PyroMain]);
    }
}
