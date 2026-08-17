//! [`FlightEstimators`] — both flight estimators and the policy connecting
//! them, in one struct, so firmware holds ONE thing behind ONE mutex.
//!
//! Composition philosophy: the two estimators are fully independent — zero
//! shared state, different clocks, different consumers. They do both read
//! the accelerometer, but only ever the raw vector off the wire, and for
//! unrelated purposes; neither can see what the other made of it.
//!
//! * The **deployment** half ([`RocketStateEstimator`]) is baro-driven, and
//!   its output is trusted outright — it fires the pyros. It never reads
//!   anything from the airbrakes half. The one non-baro input it takes is
//!   the raw accelerometer, feeding a magnitude check that can only make
//!   ignition detection *earlier* than the baro pair would (see
//!   [`FlightProfile::ignition_detection_acc_threshold`]); everything after
//!   that instant is barometric. Its *filter* is sample-clocked on
//!   purpose (a fixed `DT` step per sample, so no clock can surprise it);
//!   its *timers* read the sample timestamp, so a lockout configured as
//!   26 s lasts 26 seconds and not 26 seconds' worth of nominal ticks.
//! * The **airbrakes** half ([`AirbrakesEstimator`]) is IMU+baro and
//!   wall-clock throughout: every integration step, every window and every
//!   sustain timer uses the measured dt between the sample timestamps it is
//!   handed, so sensor stalls and skipped samples are integrated honestly.
//!   It is accuracy-only: its output feeds the MPC, never the pyros.
//!
//! Both halves therefore take their time from the one `timestamp_us`
//! handed to [`FlightEstimators::update`], and neither has to be told what
//! rate the sensors actually run at.
//!
//! **Exactly one thing crosses between the halves**, in
//! [`FlightEstimators::update`]: the deployment estimator's apogee is one
//! of the three conditions that retire the airbrakes half. It only ever
//! *ends* the airbrakes window, never extends or informs it, and it flows
//! one way — nothing the airbrakes half computes can reach the pyros.
//!
//! There are deliberately no `&mut` component accessors, so nothing beyond
//! the read above can be wired up from outside this module.
//!
//! Failure direction of the gate: every clause of
//! [`FlightEstimators::airbrakes_mpc_states`] fails toward `None` — if
//! anything is missing, stale, or out of range, the brakes stay shut.
//! Recovery (the pyro path) does not depend on the airbrakes half at all.

use core::f32::consts::FRAC_PI_2;

use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use nalgebra::Vector2;

use crate::airbrakes_estimator::{AirbrakesConfig, AirbrakesEstimator, ImuSample};
use crate::baro_gate::BaroGateOutcome;
use crate::baro_state_estimator::{FlightProfile, RocketState, RocketStateEstimator};
use crate::utils::approximate_speed_of_sound;

/// The MPC's input state, handed out by
/// [`FlightEstimators::airbrakes_mpc_states`] exactly when the airbrakes
/// are permitted to open. Permission and state availability are one
/// `Option` — "permitted but no state" cannot be expressed.
#[derive(Debug, Clone)]
pub struct AirbrakesMPCStates {
    /// Altitude ASL (m), from the airbrakes filter.
    pub altitude_asl: f32,
    /// Velocity `[horizontal, vertical]` (m/s), from the airbrakes filter.
    pub velocity: Vector2<f32>,
}

/// Everything [`FlightEstimators`] is configured with, in one value.
///
/// The two halves stay independent at runtime — [`FlightEstimators::new`]
/// hands each estimator only its own field and nothing crosses afterwards.
/// There is no invariant between the two halves to check: "never under
/// thrust" is a property of the airbrakes state machine itself, which
/// refuses to birth the vertical filter before its own measured burnout
/// latch. So this is a plain pair: somewhere for firmware to write both
/// halves down, and one value for [`FlightEstimators::new`] to take.
#[derive(Debug, Clone)]
pub struct FlightConfig {
    /// The deployment estimator's profile: Mach lockout, burn timer, and
    /// the drogue/main deployment scheme. This half fires the pyros.
    pub profile: FlightProfile,
    /// The airbrakes estimator's config, including the airframe it shares
    /// with the MPC.
    pub airbrakes: AirbrakesConfig,
}

/// The two flight estimators plus the policy connecting them. See the
/// module docs for the composition philosophy.
#[derive(Debug)]
pub struct FlightEstimators {
    deployment: RocketStateEstimator,
    /// `None` once the airbrakes window has closed for good — see
    /// [`FlightEstimators::update`]. Retirement is destructive on purpose:
    /// there is no state left to re-open the brakes from, so the window
    /// cannot reopen no matter what any later sample looks like.
    airbrakes: Option<AirbrakesEstimator>,
}

impl FlightEstimators {
    pub fn new(config: FlightConfig) -> Self {
        // Each half gets only its own half of the config; nothing is shared
        // past this point.
        Self {
            deployment: RocketStateEstimator::new(config.profile),
            airbrakes: Some(AirbrakesEstimator::new(config.airbrakes)),
        }
    }

    /// The ONLY mutating function — call once per sensor sample, with the
    /// timestamp that sample was taken at (us, one monotonic clock).
    ///
    /// Baro is always present: the deployment estimator's KF steps once per
    /// call and must see every sample. IMU is optional: when `imu` is
    /// `None` the airbrakes estimator is skipped entirely for this sample —
    /// its measured-dt integration bridges the gap at the next IMU sample.
    ///
    /// Returns the deployment estimator's pyro command passed through
    /// UNTOUCHED — this struct adds no policy to recovery — paired with
    /// [`EstimatorLogSample`]: everything a consumer wants from this sample,
    /// which is the SD log's whole estimator half and every estimator field
    /// the telemetry packet carries.
    ///
    /// The log sample is a return value rather than something read back
    /// afterwards because the baro gate outcomes it carries describe THIS
    /// sample and nothing keeps them: there is no accessor to call late, and
    /// therefore no timing contract to get wrong. Slower consumers read the
    /// published sample rather than re-reading the estimators on their own
    /// clock, which is the point — the 5 Hz downlink and the per-sample SD
    /// record are then literally the same values, not two code paths that
    /// agree by inspection.
    ///
    /// `#[must_use]` on the whole tuple, and deliberately: `Option` is not
    /// itself a `#[must_use]` type (verified — rustc warns on neither a bare
    /// `Option` return nor one inside a tuple), so putting the pyro command
    /// in a tuple would otherwise have left dropping it entirely silent. That
    /// would be catastrophic: this is the only place a drogue or main command
    /// exists.
    ///
    /// This is also where the airbrakes half is **retired**: dropped
    /// outright, never to return, as soon as any of three things is true.
    /// Two are the airbrakes estimator's own reading of the airframe, and
    /// one is the deployment estimator's:
    ///
    /// * vertical velocity at or below zero — the rocket has stopped
    ///   climbing, so there is no apogee left to shape;
    /// * the rocket is pointing below the horizon (tilt past 90 deg) —
    ///   whatever the filter thinks its velocity is, drag is no longer
    ///   acting along the axis the MPC's model assumes;
    /// * the deployment estimator has called apogee — the trusted half,
    ///   and the backstop for the airbrakes filter being wrong or stuck
    ///   about either of the above.
    ///
    /// Dropping rather than gating is the point: a flag can be misread and
    /// a gate can be bypassed by a later clause, but there is no way to
    /// hand out MPC states from an estimator that no longer exists.
    #[must_use]
    pub fn update(
        &mut self,
        timestamp_us: u64,
        imu: Option<&ImuSample>,
        baro_altitude_asl: f32,
    ) -> (Option<PyroSelect>, EstimatorLogSample) {
        // (a) Deployment first, trusted outright. Its pyro command is
        // returned as-is at the bottom.
        //
        // It gets the RAW accelerometer vector, straight off the wire —
        // never anything the airbrakes half derived from it. That half is
        // not consulted here and is not even constructed yet at the only
        // moment this value matters. See
        // `FlightProfile::ignition_detection_acc_threshold` for what it
        // does with it, which is one magnitude check and nothing else.
        let (pyro, deployment_baro_gate) =
            self.deployment
                .update(timestamp_us, imu.map(|imu| imu.acc), baro_altitude_asl);

        // (b) Airbrakes, only when this sample actually carries IMU data.
        //
        // The gate outcome is SYNTHESISED on every other sample rather than
        // carried over: no IMU means the vertical filter did not step and no
        // baro was fused, so nothing was rejected, and `Accepted` is what
        // every "no gate ran" path in either estimator already reports. The
        // stored field this replaced would instead have repeated the previous
        // sample's outcome into the SD record — a rejection run would have
        // read one sample longer than it was for every IMU-less sample inside
        // it.
        let airbrakes_baro_gate = match (self.airbrakes.as_mut(), imu) {
            (Some(airbrakes), Some(imu)) => {
                airbrakes.update(timestamp_us, imu, baro_altitude_asl)
            }
            _ => BaroGateOutcome::Accepted,
        };

        // (c) Retirement. Checked every sample, IMU or not, so clause 3
        // still bites while the airbrakes half is starved of IMU data.
        if let Some(airbrakes) = self.airbrakes.as_ref() {
            let descending = airbrakes.velocity().is_some_and(|v| v.y <= 0.0);
            let below_horizon = airbrakes.tilt().is_some_and(|t| t >= FRAC_PI_2);
            let deployment_apogee = !matches!(
                self.deployment.state(),
                RocketState::OnPad | RocketState::Ascent { .. } | RocketState::MachLockout { .. }
            );
            if descending || below_horizon || deployment_apogee {
                log_info!(
                    "retiring airbrakes estimator (descending: {}, below horizon: {}, deployment apogee: {})",
                    descending,
                    below_horizon,
                    deployment_apogee
                );
                self.airbrakes = None;
            }
        }

        // (d) The log sample, built AFTER retirement so that the sample the
        // airbrakes half is dropped on already reports the whole airbrakes
        // group absent — the same instant the SD record and the downlink go
        // absent, with nothing in between.
        let log_sample = EstimatorLogSample {
            deployment_altitude_asl: self.deployment.kf_altitude_asl(),
            deployment_vertical_velocity: self.deployment.kf_vertical_velocity(),
            deployment_launch_pad_altitude_asl: self.deployment.launch_pad_altitude_asl(),
            deployment_baro_gate,
            airbrakes: self.airbrakes.as_ref().map(|ab| AirbrakesLogSample {
                altitude_asl: ab.altitude_asl(),
                vertical_velocity: ab.velocity().map(|v| v.y),
                tilt_rad: ab.tilt(),
                subsonic_by_drag: ab.subsonic_by_drag(),
                burnout_detected: ab.burnout_detected(),
                baro_trusted: ab.baro_trusted(),
                baro_gate: airbrakes_baro_gate,
                calibration_complete: ab.calibration_complete(),
            }),
        };

        (pyro, log_sample)
    }

    /// `Some` exactly when the airbrakes are permitted to open, carrying
    /// the MPC's input state. Permission and state are one `Option`, so
    /// "permitted but no state" cannot be expressed — the MPC's run/stop
    /// condition and its state source are the same value.
    ///
    /// The gate, in order:
    /// * the airbrakes half has not been retired — everything about *when
    ///   the window ends* lives in [`Self::update`], not here;
    /// * the filter is alive — baro trusted, and its altitude and velocity
    ///   exist. "Never under thrust" is folded into this: the filter cannot
    ///   be born before the estimator's own axial-sign burnout latch, on
    ///   either the supersonic or the subsonic path, so a separate coasting
    ///   clause would be redundant;
    /// * vertical velocity at most the airframe's configured
    ///   [`max_open_mach`] of the local speed of sound at the filter's own
    ///   altitude — the same Mach the lockout-exit drag check votes at, read
    ///   back out of the airbrakes half because this gate carries no config
    ///   of its own.
    ///
    /// That last clause is normally slack: the check needs 1 s of sustain
    /// and cannot speak before `earliest_subsonic_after_ignition_us`, so
    /// birth lands well under the threshold rather than at it (Osiris 0.726,
    /// LC'25 0.727). It earns its place on a Cd model that overestimates
    /// drag, where the inverted airspeed reads low and births the filter
    /// early — measured at Mach 0.90 on an LC'25 replay with a 2x Cd error.
    /// The dead reckoner behind this clause does not depend on Cd, which is
    /// why it can disagree with the check at all.
    ///
    /// Every clause here is evaluated on the airbrakes filter's OWN state —
    /// never the slow filter's, which may be frozen (Mach lockout) or
    /// lagging hundreds of metres during coast. Any clause failing yields
    /// `None`: the brakes stay shut.
    ///
    /// [`max_open_mach`]: crate::airbrakes_estimator::AirbrakesConfig::max_open_mach
    pub fn airbrakes_mpc_states(&self) -> Option<AirbrakesMPCStates> {
        let airbrakes = self.airbrakes.as_ref()?;
        if !airbrakes.baro_trusted() {
            return None;
        }
        let altitude_asl = airbrakes.altitude_asl()?;
        let velocity = airbrakes.velocity()?;

        if velocity.y > airbrakes.max_open_mach() * approximate_speed_of_sound(altitude_asl) {
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

    /// The pad altitude the deployment half is holding (m ASL) — the one
    /// reference every AGL number is measured from, on the downlink and in
    /// the MPC's apogee target alike.
    ///
    /// This used to be reached through a `deployment_estimator()` accessor
    /// that handed out the whole `&RocketStateEstimator`. One f32 is all
    /// firmware ever wanted from it, and handing out only the f32 is what
    /// makes the alternative unwritable: the KF's altitude and velocity are
    /// reachable only through the [`EstimatorLogSample`] that
    /// [`Self::update`] returns, once per sample, so no consumer can re-read
    /// the filter on a clock of its own and disagree with the SD log about
    /// what a sample said.
    ///
    /// Not an `Option`, unlike the filter's numbers: this is a low-passed
    /// barometer reading while the rocket is on the rail and a constant
    /// latched at ignition detection afterwards, so it exists in every
    /// stage including the Mach lockout, where the filter itself is frozen.
    /// It reads 0.0 only before the first sample anchors it (see
    /// [`RocketStateEstimator::launch_pad_altitude_asl`]).
    pub fn launch_pad_altitude_asl(&self) -> f32 {
        self.deployment.launch_pad_altitude_asl()
    }

    /// Read-only access to the airbrakes estimator (drag check, birth, tilt,
    /// fast-record flag assembly, ...).
    ///
    /// Deliberately no `&mut` twin: cross-estimator data flows by value,
    /// inside [`Self::update`], once per sample — the API makes any other
    /// coupling between the two estimators impossible to write.
    ///
    /// `None` once retired (see [`Self::update`]), which callers should
    /// treat as "no reading" rather than "reading of zero" — the telemetry
    /// and SD-log fields sourced from here go absent from that sample on.
    pub fn airbrakes_estimator(&self) -> Option<&AirbrakesEstimator> {
        self.airbrakes.as_ref()
    }

}

/// One sample's worth of estimator state for the SD log and the downlink,
/// returned by [`FlightEstimators::update`].
#[derive(Debug, Clone, Copy)]
pub struct EstimatorLogSample {
    /// The deployment KF's output, `None` on every sample where that filter
    /// has no live reading: before it is born, and throughout the Mach
    /// lockout, where it is frozen (see
    /// [`RocketStateEstimator::kf_altitude_asl`]). Absent, not zero and not
    /// the stale frozen value — so this record and the telemetry packet,
    /// which sources the same window from [`RocketState::MachLockout`],
    /// agree that nothing was measured there.
    pub deployment_altitude_asl: Option<f32>,
    pub deployment_vertical_velocity: Option<f32>,
    /// The pad altitude the deployment half is holding (m ASL), carried so
    /// that a consumer of this sample can turn the ASL above into AGL
    /// without reaching back into the estimators — see
    /// [`FlightEstimators::launch_pad_altitude_asl`], which is the same
    /// number and exists for the callers that already hold the lock.
    ///
    /// Not an `Option`: it exists in every stage, including the Mach lockout
    /// where the two fields above go absent.
    pub deployment_launch_pad_altitude_asl: f32,
    pub deployment_baro_gate: BaroGateOutcome,
    /// `None` once the airbrakes half is retired at apogee — absent, not zero.
    pub airbrakes: Option<AirbrakesLogSample>,
}

/// The airbrakes half of [`EstimatorLogSample`]. The `Option` fields are
/// absent until the piece of the estimator that produces them is alive.
#[derive(Debug, Clone, Copy)]
pub struct AirbrakesLogSample {
    pub altitude_asl: Option<f32>,
    pub vertical_velocity: Option<f32>,
    pub tilt_rad: Option<f32>,
    pub subsonic_by_drag: Option<bool>,
    pub burnout_detected: bool,
    pub baro_trusted: bool,
    /// No `is_apogee` twin: the airbrakes half has no apogee state to report
    /// from. It is retired at apogee instead (see
    /// [`FlightEstimators::update`]), and this whole struct goes absent on
    /// that sample — which is the same information, dated to the same tick,
    /// and reaches the SD log as the airbrakes group disappearing rather than
    /// as a bit that flips.
    pub baro_gate: BaroGateOutcome,
    /// The pad calibration exists (see
    /// [`AirbrakesEstimator::calibration_complete`]).
    ///
    /// The only field here that says something while the rocket is still on
    /// the rail, and the reason it is logged: without a calibration the
    /// estimator refuses to detect ignition, so the airbrakes silently do
    /// not fly. Every other flag below is retrospective.
    ///
    /// [`AirbrakesEstimator::calibration_complete`]:
    ///     crate::airbrakes_estimator::AirbrakesEstimator::calibration_complete
    pub calibration_complete: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::baro_state_estimator::{DT, SAMPLES_PER_S};
    use crate::tests::fixtures::{lc25_airbrakes, subsonic_profile};
    use nalgebra::Vector3;

    /// Nominal 416 Hz sample spacing in microseconds.
    const SAMPLE_DT_US: u64 = 2404;

    // Both tests below are about the *composition* — the pad gate, and the
    // pyro pass-through — so neither reads a single number out of the
    // config. They take the shared bases whole and override nothing; see
    // [`crate::tests::fixtures`].

    /// On the pad — before ignition, before the burn timer — the gate
    /// must hand out nothing, even with a healthy IMU feed coming in.
    #[test]
    fn no_mpc_states_before_coasting() {
        let mut est = FlightEstimators::new(FlightConfig {
            profile: subsonic_profile(),
            airbrakes: lc25_airbrakes(),
        });
        let imu = ImuSample {
            acc: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::zeros(),
        };

        let mut t_us = 0u64;
        for _ in 0..(5 * SAMPLES_PER_S) {
            let (pyro, _log) = est.update(t_us, Some(&imu), 200.0);
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
        // The bare half and the composed one must be handed the SAME
        // profile — that is what makes the comparison below meaningful —
        // so both take the fixture rather than two hand-written copies.
        let mut bare = RocketStateEstimator::new(subsonic_profile());
        let mut composed = FlightEstimators::new(FlightConfig {
            profile: subsonic_profile(),
            airbrakes: lc25_airbrakes(),
        });

        // Clean point-mass trajectory: 5 s pad hold, 3 s burn at
        // 80 m/s^2, ballistic coast over apogee, -25 m/s terminal
        // descent, 8 s on the ground (landed detection needs 5 s still).
        let pad_altitude_asl = 200.0f32;
        let mut samples: Vec<f32> = Vec::new();
        // Axial specific force per sample, since ignition detection is the
        // accelerometer's job: 1 g held on the rail, thrust plus 1 g under
        // power, nothing while ballistic, 1 g again once the descent is
        // aerodynamically supported.
        let mut specific_force: Vec<f32> = Vec::new();
        samples.extend(core::iter::repeat(pad_altitude_asl).take(5 * SAMPLES_PER_S));
        specific_force.extend(core::iter::repeat(9.81).take(5 * SAMPLES_PER_S));
        let mut altitude_asl = pad_altitude_asl;
        let mut velocity = 0.0f32;
        let mut t = 0.0f32;
        loop {
            let acceleration = if t < 3.0 { 80.0 } else { -9.81 };
            velocity = (velocity + acceleration * DT).max(-25.0);
            altitude_asl += velocity * DT;
            t += DT;
            if altitude_asl <= pad_altitude_asl {
                break;
            }
            samples.push(altitude_asl);
            specific_force.push(if t < 3.0 {
                80.0 + 9.81
            } else if velocity <= -25.0 {
                9.81
            } else {
                0.0
            });
        }
        samples.extend(core::iter::repeat(pad_altitude_asl).take(8 * SAMPLES_PER_S));
        specific_force.extend(core::iter::repeat(9.81).take(8 * SAMPLES_PER_S));

        // Both halves get the same stream; the deployment half (the one
        // under test here) is what the assertions compare.
        let mut fires = Vec::new();
        for (i, (&alt, &sf)) in samples.iter().zip(specific_force.iter()).enumerate() {
            let t_us = i as u64 * SAMPLE_DT_US;
            let acc = Some(Vector3::new(0.0, 0.0, sf));
            let (expected, _expected_gate) = bare.update(t_us, acc, alt);
            let imu = ImuSample {
                acc: acc.unwrap(),
                gyro: Vector3::zeros(),
            };
            let (got, _log) = composed.update(t_us, Some(&imu), alt);
            assert_eq!(expected, got, "pyro mismatch at sample {i}");
            if let Some(pyro) = got {
                fires.push(pyro);
            }
        }

        // Single deployment: drogue at apogee, main on the next sample.
        assert_eq!(fires, vec![PyroSelect::PyroDrogue, PyroSelect::PyroMain]);
    }
}
