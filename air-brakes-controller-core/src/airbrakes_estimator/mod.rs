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
//! # Four states, and it only walks forward
//!
//! `Armed` -> `Stage1` -> `DeadReckoning` -> `AirbrakesEnabled`. Nothing goes
//! back, nothing skips, and there is no fifth: the estimator's life ends by
//! being dropped whole at apogee, not by transitioning. Armed screens the pad
//! for a calibration and watches for ignition; Stage1 is the first half second
//! of thrust, which is what solves how the avionics are mounted; DeadReckoning
//! is boost and the Mach lockout, inertial only, buffering the baro without
//! fusing it; AirbrakesEnabled is the vertical filter running, and the brakes
//! permitted to open, until the drop.
//!
//! Everything that decides whether the brakes may open is therefore a
//! transition condition, not a live one. That is the point rather than a
//! detail of the encoding: the Mach limit used to be re-tested downstream on
//! every sample against the filter's own velocity, and so could withdraw a
//! permission it had already granted — the filter's birth transient did
//! exactly that, shutting the brakes 25 ms after opening them and reopening
//! them 170 ms later. Asked once, on the way in, of the dead-reckoned
//! velocity the filter is about to be born with, it answers the question it
//! exists for and cannot be re-answered by a filter that is briefly wrong.
//!
//! # The lockout exit is one measurement
//!
//! In free flight the accelerometer measures specific force, which
//! excludes gravity — so its component along the airframe axis IS
//! drag/mass, and the sign of that same component is what says the motor is
//! out. One channel answers both questions; it was a magnitude and a
//! separate axial test until 2026-08-17, and the magnitude was the half
//! that could not tell thrust from drag. Inverting
//! `a = 0.5 * rho * v^2 * Cd*A/m` therefore yields the airspeed with **no
//! integration, no attitude, and no baro**: nothing in it can drift, and
//! nothing in it can be poisoned by the very static-port error the lockout
//! exists to wait out. The one atmospheric input, air density, is not
//! measured at all — it is evaluated at
//! [`MachLockoutConfig::subsonic_crossing_altitude_asl`], the sim's altitude
//! for this crossing, which is what keeps the "no integration, no baro"
//! claim literally true rather than nearly true.
//!
//! It crosses Mach 0.8 within 0.2 s of the inertial estimate while never
//! once dipping below the threshold during the supersonic phase.
//!
//! Two conditions it depends on, both enforced by [`MachLockoutConfig`]:
//! the motor must be out (thrust tail-off briefly cancels drag, which
//! reads as a false low speed), and the brakes must be stowed (they are —
//! this gate is what opens them).
//!
//! Every step uses the measured time between samples (each sample carries
//! its timestamp) — nothing here assumes a sample rate, including the
//! pieces that are not integrations: the 2 s pad calibration windows, the
//! pre-ignition rewind buffer, the ignition low pass, and every sustain
//! timer are all spans of measured time. Since 2026-08-17 that is literally
//! true and not a near miss: no nominal rate survives in the flight path at
//! all, because the very first sample — the one with no predecessor to
//! difference against, and the last thing that assumed one — is stepped by
//! dt = 0 rather than by an assumed 1/416 s. The flight log that motivated
//! this had 104 ms sensor stalls, and the part whose ODR is written
//! "416 Hz" measures 427.02 Hz.

use nalgebra::Vector3;

use crate::controller::RocketParameters;

mod dead_reckoner;
mod estimator;
#[cfg(test)]
mod tests;
mod vertical_kf;

pub use estimator::AirbrakesEstimator;

/// Per-sample dt clamp: a gap longer than this is integrated as this long
/// (protects the integrators from a bogus timestamp jump). Long enough to
/// integrate honestly through the measured 104 ms stalls.
pub(crate) const MAX_DT_S: f32 = 0.25;

/// One IMU sample in the avionics (IMU chip) frame, SI units at the API
/// boundary: specific force in m/s^2, angular velocity in rad/s. Firmware
/// converts units at the edge (e.g. deg/s -> rad/s) before constructing
/// this.
///
/// Acc and gyro must share one consistent right-handed frame; the estimator
/// self-calibrates the mounting orientation on the pad, so no per-board axis
/// configuration is needed.
///
/// The timestamp and the baro altitude that go with it are passed alongside
/// rather than folded in here, because that is the shape both callers
/// already have: [`FlightEstimators::update`] takes one timestamp for BOTH
/// halves and a baro reading the deployment half also needs, and it holds
/// the IMU as an `Option` because a sample may carry baro without IMU. A
/// combined "one timestamped IMU+baro sample" type stood here until
/// 2026-08-17 and only ever moved those three values from that call
/// straight into this estimator's `update`.
///
/// [`FlightEstimators::update`]: crate::FlightEstimators::update
#[derive(Debug, Clone)]
pub struct ImuSample {
    /// Accelerometer specific force (m/s^2).
    pub acc: Vector3<f32>,
    /// Angular velocity (rad/s).
    pub gyro: Vector3<f32>,
}

/// Airbrakes estimator configuration. All numbers are per-airframe /
/// per-motor and come from the flight simulation or prior flight data.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct AirbrakesConfig {
    /// `Some` for flights that go near or above the speed of sound: the
    /// baro is ignored from ignition until the drag check (bounded by
    /// these timers) says the flow is subsonic again. `None` for subsonic
    /// profiles: the filter is born right after the thrust-vector
    /// alignment finishes.
    pub mach_lockout: Option<MachLockoutConfig>,

    /// **The** Mach number of this airframe: below it the flow is subsonic,
    /// the static port is honest, and the flaps may open. One value, read in
    /// two places, because it is one physical fact about the rocket:
    ///
    /// * the lockout-exit drag check votes at it — the inverted airspeed
    ///   falling below it (and staying there) is what births the vertical
    ///   filter and ends the Mach lockout;
    /// * [`FlightEstimators::airbrakes_mpc_states`] refuses to hand out MPC
    ///   states while the filter's own vertical velocity is above it, which
    ///   is what actually keeps the flaps shut.
    ///
    /// Per-airframe, and no longer tied to the Mach [`Self::rocket`]'s stowed
    /// `cd[0]` is tabulated at. Those were required to be equal until
    /// 2026-08-18, on the argument that the check inverts the drag with that
    /// Cd to decide whether it is below this speed. They are now allowed to
    /// differ, and Osiris sets this ABOVE the tabulation Mach (0.83 against a
    /// Cd table taken at 0.8), because the direction of the resulting error
    /// is the safe one and the control window it buys is worth ~38 m of
    /// apogee authority.
    ///
    /// Why it is safe to set this high: inverting with a Cd measured at a
    /// LOWER Mach than the airframe is actually flying under-reads the drag
    /// coefficient, which over-reads the inverted airspeed, which holds the
    /// lockout shut LONGER. So the mismatch delays the birth rather than
    /// advancing it, and it grows with the gap — measured, the birth lands
    /// 0.015-0.021 Mach under this threshold rather than at it.
    ///
    /// The bound that replaces the old equality is therefore not a config
    /// invariant but a physical one, and it belongs to the airframe, not to
    /// the Cd table: **do not set this above the Mach the flaps and their
    /// interaction with the fins have been analysed at**. Osiris's CFD
    /// (FDR Table 10) covers Mach 0.8 and nothing else, so 0.83 is already
    /// extrapolating; the measured birth at 0.809-0.815 is ~2% outside the
    /// analysed condition and that is the whole of the margin being spent.
    /// `mach_lockout_timers_bracket_every_simulation` asserts the vote stays
    /// under this value; nothing can assert the aerodynamics.
    ///
    /// The OTHER bound, and the one that actually set 0.83 rather than a
    /// rounder number, is
    /// `osiris_sim::transonic_static_port_error_is_absorbed_by_the_lockout`.
    /// Opening earlier is being born deeper into the window where the static
    /// port is still lying, and that test's coast velocity error climbs
    /// 5.53 -> 6.68 -> 7.44 -> 8.13 m/s at 0.80 / 0.82 / 0.83 / 0.84 against
    /// its 8 m/s bound. 0.84 fails. Raising this is therefore not a
    /// judgement call: it is bounded by a test already in the tree, and the
    /// bound is a real one — simulated end to end, a port error worth ~240 m
    /// of altitude at the gate makes 0.85 WORSE than 0.80, because the extra
    /// authority is bought with robustness that the fault then spends.
    ///
    /// This IS the requirement, not the requirement minus margin. The margin
    /// arrives on its own, from two places that are already load-bearing:
    /// inverting with the SUBSONIC Cd reads high while the true Cd is
    /// transonically elevated, and
    /// [`MachLockoutConfig::earliest_subsonic_after_ignition_us`] delays the
    /// decision further. Measured, birth lands under this threshold rather
    /// than at it — LC'25 0.772 — so the velocity check is
    /// slack on a healthy flight without needing a separate, higher number
    /// to make it so.
    ///
    /// Read as an approximation with about +-0.05 Mach either side, not as a
    /// hard edge — it is an estimate of where the flaps stop being qualified,
    /// and the drag check that enforces it inverts a Cd that is itself a
    /// model. A second, Cd-independent opinion (the dead reckoner's own
    /// velocity, tested once at the birth site) enforced it more tightly
    /// until 2026-08-18; what it cost is in `estimator.rs` at the birth
    /// site, and what its removal costs is +0.01 Mach at a 0.6 ceiling,
    /// nothing at the 0.8 flown until 2026-08-18, and 0.857 rather than
    /// 0.787 if `Cd*A/m` is a third too large.
    ///
    /// This was two constants until 2026-08-17 — a 0.8 exit and a 0.85
    /// ceiling with an invariant between them. The 0.85 was never derived
    /// from anything; it was 0.8 plus margin that `t_min` and the subsonic
    /// Cd already supply. If an airframe ever turns up whose flaps are
    /// qualified *below* the Mach at which its baro recovers, split it again
    /// then — and note that limit would sit under this one, the opposite way
    /// round from the invariant the split used to carry.
    ///
    /// [`FlightEstimators::airbrakes_mpc_states`]: crate::FlightEstimators::airbrakes_mpc_states
    pub max_open_mach: f32,

    /// The airframe — the same value the MPC flies on.
    ///
    /// The drag check needs `Cd * A / m`, and it derives that here rather
    /// than taking it as a number, so the lockout and the apogee
    /// prediction cannot be given different airframes. It reads `cd[0]`,
    /// the brakes-stowed entry, and that is what makes the check one-sided:
    /// the true Cd is higher transonically, so the inverted speed reads
    /// high exactly while supersonic and the check errs toward keeping the
    /// lockout shut. Measured on LC'25, inverting the low-passed axial
    /// channel at the configured crossing altitude, the inverted Mach peaks
    /// at 1.31 where the truth is 1.03. Projecting onto the axis instead of
    /// taking `|acc|` costs 2% of that headroom (the magnitude peaks at
    /// 1.34), which buys the sign the check needs to reject thrust outright.
    pub rocket: RocketParameters,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
/// Bounds on when the drag check is allowed to decide, both measured from
/// **this estimator's own accelerometer ignition detection**.
///
/// Note the clock: [`FlightProfile::mach_lockout_duration_us`] is measured
/// from the DEPLOYMENT half's ignition detection. Since 2026-08-17 that is
/// the same detector this half runs — one implementation
/// ([`crate::ignition_detector`]), two instances, same 10 Hz low pass on the
/// raw accelerometer and same 0.1 s sustain — and since 2026-08-18 the same
/// threshold too: it lives once, in
/// [`FlightConfig::ignition_detection_acc_threshold`], and is handed to both
/// detectors at construction. No parameter can separate the two origins any
/// more; `osiris_sim::ignition_latch_time_by_threshold` sweeps that one
/// number and asserts the two halves never disagree.
///
/// One thing still can separate them in flight, and it never was a
/// threshold: this half refuses to detect ignition until its pad calibration
/// completes (three 2 s windows), so a board armed seconds before liftoff
/// gives this half a later origin, or none, while the pyro half's is
/// unaffected. That is why the halves hold separate instances rather than
/// sharing one.
///
/// What remains independent is the two lockouts themselves — different
/// subsystems, different exit conditions, and different Mach numbers
/// (0.75 there, [`AirbrakesConfig::max_open_mach`] here). Equal values in a
/// config are still coincidence, not a relationship; changing one does not
/// imply changing the other. The ignition threshold is the one thing that
/// was never independent, which is why it stopped being two fields.
///
/// [`FlightProfile::mach_lockout_duration_us`]: crate::FlightProfile::mach_lockout_duration_us
/// [`FlightConfig::ignition_detection_acc_threshold`]:
///     crate::FlightConfig::ignition_detection_acc_threshold
pub struct MachLockoutConfig {
    /// Earliest the rocket could possibly be below
    /// [`AirbrakesConfig::max_open_mach`]; the drag check is not consulted
    /// before this.
    ///
    /// A hard floor, and load-bearing against exactly one kind of error.
    /// Measured on the Osiris sim, a *config* Cd wrong by 5x moves the birth
    /// only from 18.72 s to 18.64 s — against a true Mach 0.8 crossing at
    /// 17.56 s — because this floor sits at 17.5 s and the check simply
    /// cannot speak earlier. Note what that sweep varies, though: the config
    /// moves and the trajectory does not, so the floor keeps its measured
    /// distance from the real crossing by construction. Against a wrong
    /// *true* Cd the airframe flies a different flight and the crossing
    /// moves out from under the floor, which this constant does nothing
    /// about — it is a fixed time, not a measurement.
    ///
    /// The constant that is load-bearing in both cases is the 1 s subsonic
    /// sustain (`estimator::SUBSONIC_SUSTAIN_S`): removing that alone takes
    /// the share of births still above Mach 0.8 from 1.0% to 45.4%. What
    /// this floor uniquely covers is the other half of the safety argument —
    /// the burnout latch fires during tail-off with several hundred newtons
    /// still on the case, and residual thrust cancels drag, so a check
    /// allowed to speak then would invert an unrealistically low
    /// deceleration into an unrealistically low speed. At 17.5 s the motor
    /// has been out for eleven seconds; `osiris_sim::nominal_o3400_flight`
    /// asserts that margin.
    ///
    /// Set it loosely and a wrong drag model births the filter while the
    /// port is still shocked: LC'25's floor is ~2.9 s ahead of its real
    /// crossing (ignition+8.0 s against +10.90 s), and a 2x Cd error there
    /// births at Mach 0.887.
    ///
    /// From the flight sim: the earliest simulated time below that Mach,
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
    /// From the flight sim: the latest plausible time below
    /// [`AirbrakesConfig::max_open_mach`] plus
    /// margin, and it must end well (>5 s) before the EARLIEST simulated
    /// apogee — a forced birth after apogee leaves the airbrakes no window
    /// at all.
    ///
    /// This backstop is still subject to the burnout latch: it covers a
    /// drag model wrong enough that the check never passes (the axial sign
    /// test does not depend on Cd, so the latch still fires), not an
    /// accelerometer too dead to show deceleration at all.
    pub force_birth_after_ignition_us: u32,

    /// Altitude ASL (m) the airframe is expected to be at when it crosses
    /// [`AirbrakesConfig::max_open_mach`] on the way down — the altitude the
    /// drag check evaluates air density and the speed of sound at.
    ///
    /// From the flight sim, exactly like the two timers above and from the
    /// same run: the altitude at the coast-side crossing of that Mach. It is
    /// a constant rather than a measurement because the check only has to be
    /// right in one narrow window — the second or so around that crossing —
    /// and inside that window the airframe is, by construction, at this
    /// altitude.
    ///
    /// Being a constant is the point, and it buys two things a tracked
    /// altitude cannot. It cannot drift, since nothing integrates it. And it
    /// cannot be poisoned by the static port, which is the failure the whole
    /// lockout exists to wait out — so the decision "is the baro honest yet?"
    /// stays independent of the baro. Until 2026-08-17 this came from the
    /// dead reckoner's own integrated altitude, which had the second property
    /// but not the first.
    ///
    /// **Erring high is the safe direction, so round up.** Density falls with
    /// altitude, so an altitude set too high reads `rho` too low, and the
    /// inverted airspeed `sqrt(2a / (rho * CdA/m))` too HIGH — which keeps
    /// the lockout shut longer. Set it from the highest crossing the
    /// airframe's motors plausibly produce, not the average.
    ///
    /// The number that decides is the TRUE Mach the check ends up voting at,
    /// which folds in both altitude-dependent terms — density, and the speed
    /// of sound the threshold is scaled by:
    ///
    /// ```text
    /// M_vote = max_open_mach * a(h_cfg)/a(h_true) * sqrt(rho(h_cfg)/rho(h_true))
    /// ```
    ///
    /// Every motor that crosses below `h_cfg` drives both factors under 1 and
    /// votes conservatively. Measured across Osiris's two motors against the
    /// configured 6800 m: the O3400 crosses at 6734 m and votes at Mach
    /// 0.796, the N2900 backup crosses at 5583 m and votes at 0.736 — both
    /// under the configured 0.800.
    /// `osiris_sim::mach_lockout_timers_bracket_every_simulation` computes
    /// exactly this and asserts it for every simulation in the archive, which
    /// is what stops a new motor from being added without revisiting this
    /// constant.
    ///
    /// The sensitivity is mild enough that the sim's number needs no margin
    /// beyond rounding up: 1151 m of altitude error is worth 6.6% of
    /// airspeed, against the 1 s subsonic sustain and the
    /// [`Self::earliest_subsonic_after_ignition_us`] floor that already delay
    /// the decision. What it is NOT tolerant of is being left at the pad
    /// altitude: that is a 2x density error on this airframe, reads the
    /// airspeed ~30% low, and births the filter while genuinely supersonic on
    /// the LC'25 replay (measured — it births at ignition+10.7 s against a
    /// true crossing at +10.90 s).
    pub subsonic_crossing_altitude_asl: f32,
}
