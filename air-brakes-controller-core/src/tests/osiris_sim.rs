//! Osiris (CTI O3400) flown against both estimators and the MPC, from the
//! OpenRocket simulations in `2026_06_26 - Osiris LC FDR.ork`.
//!
//! # Why this file exists
//!
//! The other replays ([`crate::airbrakes_estimator::tests`]) are RAW flight
//! logs: real sensors, real noise, real stalls — and no truth to check
//! against beyond the baro itself. This file is the mirror image. There is
//! no real Osiris log yet, so the sensors are synthesised; but the
//! trajectory is a full 6-DOF OpenRocket run, which means every answer the
//! estimator produces can be scored against a known truth: true apogee,
//! true vertical velocity, true Mach.
//!
//! It exists to answer one question before the rocket flies: does the
//! flight config in `VLF5/firmware/src/main.rs` actually work on THIS
//! airframe and THIS motor?
//!
//! # What is truth and what is invented
//!
//! From OpenRocket, per row: time, altitude AGL, vertical/lateral velocity,
//! zenith and azimuth of the airframe, mass, thrust, drag, static pressure,
//! density, speed of sound, Mach, local gravity. That is the whole
//! trajectory — nothing about it is guessed.
//!
//! Invented here, and only here:
//!
//! * **Sensor noise, bias and quantisation.** Sized from the Void Lake pad
//!   segment (see [`SensorModel`] for the measured numbers), and quantised
//!   to the LSM6DSM/MS5607 LSBs the drivers actually configure.
//! * **Roll.** OpenRocket reports roll rate identically zero for this
//!   design, which is not a rocket. A spin-up/decay profile peaking at
//!   1 rev/s is added, and the accelerometer and gyro are both generated
//!   from the same rolling attitude.
//! * **The IMU mounting orientation.** A fixed, deliberately ugly rotation
//!   between the airframe and the chip, so the estimator's pad
//!   self-calibration has something to find.
//! * **Pad time.** The sim starts at ignition; the rocket sits armed on the
//!   rail for minutes. A quiet pad segment (with a little rail sway) is
//!   prepended.
//! * **The transonic static-port error**, in the tests that ask for it.
//!   OpenRocket reports the true freestream static pressure; a real port on
//!   a Mach 1.9 airframe does not. See [`SensorModel::transonic_port_error`].
//!
//! The attitude model is not free-floating either: [`orientation_model_matches_openrocket`]
//! checks the body rates it produces against OpenRocket's own reported
//! pitch/yaw rates, and [`sensor_model_matches_openrocket_forces`] checks
//! the synthesised specific force against OpenRocket's own thrust and drag.
//! If either drifts, every number below is worthless and the test says so.

use core::f32::consts::PI;

use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::{UnitQuaternion, Vector2, Vector3};

use crate::airbrakes_estimator::{AirbrakesConfig, ImuSample, MachLockoutConfig};
use crate::baro_state_estimator::{DeploymentProfile, FlightProfile};
use crate::controller::{AirBrakesMPC, RocketParameters};
use crate::flight_estimators::{FlightConfig, FlightEstimators};
use crate::tests::init_logger;
use crate::utils::{approximate_air_density, approximate_speed_of_sound};

// ---------------------------------------------------------------------------
// The flight config under test
// ---------------------------------------------------------------------------

/// The airframe as flown, copied from `VLF5/firmware/src/main.rs`
/// (`FLIGHT_CONFIG.airbrakes.rocket`). `cd` is the STAR-CCM+ table from FDR
/// Table 10 converted to coefficients; `reference_area` is OpenRocket's own
/// reference area for this airframe, which the CSVs carry in
/// `reference_area_m2` — [`config_matches_the_simulated_airframe`] checks
/// the two against each other.
fn osiris_rocket() -> RocketParameters {
    RocketParameters {
        burnout_mass: 18.696,
        cd: [0.61365, 0.69816, 0.8084, 0.96641, 1.12441],
        reference_area: 0.009854945,
    }
}

/// `FLIGHT_CONFIG` from `VLF5/firmware/src/main.rs`, verbatim.
///
/// Duplicated rather than imported because that constant lives in the
/// firmware crate, which does not build for the host. Everything the
/// numbers claim about the trajectory is asserted against the simulations
/// in [`mach_lockout_timers_bracket_every_simulation`], so a copy that
/// drifts out of date fails loudly rather than quietly passing.
fn osiris_config() -> FlightConfig {
    FlightConfig {
        profile: FlightProfile {
            mach_lockout_duration_us: Some(26_000_000),
            // Mirrors VLF5's `FLIGHT_CONFIG`. Both halves run the same
            // detector at the same 8 g — see `crate::ignition_detector`.
            ignition_detection_acc_threshold: 8.0 * 9.81,
            deployment: DeploymentProfile::Dual {
                drogue_chute_minimum_altitude_agl: 2000.0,
                drogue_chute_delay_us: 1_000_000,
                main_chute_altitude_agl: 457.2,
                main_chute_delay_us: 0,
            },
        },
        airbrakes: AirbrakesConfig {
            ignition_detection_acc_threshold: 8.0 * 9.81,
            mach_lockout: Some(MachLockoutConfig {
                earliest_subsonic_after_ignition_us: 17_500_000,
                force_birth_after_ignition_us: 25_000_000,
                subsonic_crossing_altitude_asl: 6800.0,
            }),
            max_open_mach: 0.8,
            rocket: osiris_rocket(),
        },
    }
}

const O3400_CSV: &str = "./test_data/osiris_o3400.csv";
const N2900_CSV: &str = "./test_data/osiris_n2900.csv";

// The estimators never see a geometric altitude — they see whatever
// `calculate_isa_altitude` makes of the pressure, and so does every number
// they are scored against here. That is not the same as the site's real
// altitude: this launch day is 102066 Pa at a site 363.6 m ASL, which plain
// ISA reads as -63 m. Scoring against the geometric altitude instead would
// charge the estimator hundreds of metres for the atmosphere being warmer
// than standard, which is not its job to know.

// ---------------------------------------------------------------------------
// Truth: the OpenRocket trajectory
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default)]
struct TruthRow {
    t: f32,
    /// Geometric altitude above the pad, from OpenRocket. Reported only;
    /// nothing is scored against it — see [`TruthRow::altitude_asl`].
    altitude_agl: f32,
    /// Pressure altitude: the ISA altitude of this row's static pressure.
    /// This is the truth every estimator output is compared against,
    /// because it is the only altitude the barometer can express.
    altitude_asl: f32,
    vv: f32,
    lateral_velocity: f32,
    /// Angle of the airframe from the horizontal plane (OpenRocket's
    /// "vertical orientation (zenith)"), radians. Tilt from vertical is
    /// `FRAC_PI_2 - zenith`.
    zenith: f32,
    azimuth: f32,
    /// OpenRocket's own body rates, used only to validate the attitude model.
    pitch_rate: Option<f32>,
    yaw_rate: Option<f32>,
    mass: f32,
    thrust: f32,
    drag: f32,
    pressure: f32,
    density: f32,
    speed_of_sound: f32,
    mach: f32,
    gravity: f32,
    reference_area: f32,
    /// World-frame kinematic acceleration, differentiated from the
    /// reconstructed velocity vector at load time (see [`Truth::load`]).
    acc_world: Vector3<f32>,
}

struct Truth {
    rows: Vec<TruthRow>,
}

impl Truth {
    fn load(path: &str) -> Self {
        #[derive(serde::Deserialize)]
        struct CsvRow {
            time_s: f32,
            altitude_agl_m: f32,
            vv_mps: f32,
            lateral_velocity_mps: f32,
            zenith_rad: f32,
            azimuth_rad: f32,
            pitch_rate_rps: Option<f32>,
            yaw_rate_rps: Option<f32>,
            mass_kg: f32,
            thrust_n: f32,
            drag_n: f32,
            pressure_pa: f32,
            density_kgm3: f32,
            speed_of_sound_mps: f32,
            mach: f32,
            gravity_mps2: f32,
            reference_area_m2: f32,
        }

        let mut rows: Vec<TruthRow> = csv::Reader::from_path(path)
            .unwrap()
            .deserialize::<CsvRow>()
            .map(|r| r.unwrap())
            .map(|r| TruthRow {
                t: r.time_s,
                altitude_agl: r.altitude_agl_m,
                altitude_asl: calculate_isa_altitude(Pascals(r.pressure_pa as f64)).0 as f32,
                vv: r.vv_mps,
                lateral_velocity: r.lateral_velocity_mps,
                zenith: r.zenith_rad,
                azimuth: r.azimuth_rad,
                pitch_rate: r.pitch_rate_rps,
                yaw_rate: r.yaw_rate_rps,
                mass: r.mass_kg,
                thrust: r.thrust_n,
                drag: r.drag_n,
                pressure: r.pressure_pa,
                density: r.density_kgm3,
                speed_of_sound: r.speed_of_sound_mps,
                mach: r.mach,
                gravity: r.gravity_mps2,
                reference_area: r.reference_area_m2,
                acc_world: Vector3::zeros(),
            })
            .collect();
        assert!(rows.len() > 1000, "{path}: only {} rows", rows.len());

        // Kinematic acceleration, central-differenced from the world
        // velocity vector the same columns define. Doing it here rather
        // than reading OpenRocket's acceleration columns keeps the
        // accelerometer exactly consistent with the velocity and altitude
        // the rest of the model uses — the sensor stream cannot disagree
        // with the trajectory it came from.
        // `sensor_model_matches_openrocket_forces` checks the result
        // against OpenRocket's independent thrust and drag.
        //
        // The horizontal component is laid along ONE fixed azimuth, taken
        // at burnout, rather than along each row's own airframe azimuth.
        // The airframe azimuth is the direction the rocket is *tilted*,
        // and it swings by ~10 deg over the coast; hanging the drift
        // velocity off it would rotate that vector and manufacture a
        // lateral acceleration of several m/s^2 that the rocket never
        // felt. The real drift direction barely moves, so a constant is
        // both simpler and closer to the truth — and the vertical channel,
        // which is the one every estimator output depends on, is
        // untouched by the choice.
        let drift_azimuth = {
            let burnout_t = rows
                .iter()
                .filter(|r| r.thrust > 0.0)
                .map(|r| r.t)
                .fold(0.0f32, f32::max);
            rows.iter()
                .find(|r| r.t >= burnout_t)
                .map(|r| r.azimuth)
                .unwrap_or(0.0)
        };
        let (caz, saz) = (drift_azimuth.cos(), drift_azimuth.sin());
        let vel = |r: &TruthRow| {
            Vector3::new(r.lateral_velocity * caz, r.lateral_velocity * saz, r.vv)
        };
        for i in 0..rows.len() {
            let (lo, hi) = (i.saturating_sub(1), (i + 1).min(rows.len() - 1));
            let dt = rows[hi].t - rows[lo].t;
            rows[i].acc_world = if dt > 0.0 {
                (vel(&rows[hi]) - vel(&rows[lo])) / dt
            } else {
                Vector3::zeros()
            };
        }

        Self { rows }
    }

    /// Linear interpolation, clamped at both ends.
    fn at(&self, t: f32) -> TruthRow {
        let rows = &self.rows;
        if t <= rows[0].t {
            return rows[0];
        }
        if t >= rows[rows.len() - 1].t {
            return rows[rows.len() - 1];
        }
        let i = rows.partition_point(|r| r.t <= t).max(1);
        let (a, b) = (&rows[i - 1], &rows[i]);
        let s = (t - a.t) / (b.t - a.t);
        let l = |x: f32, y: f32| x + (y - x) * s;
        TruthRow {
            t,
            altitude_agl: l(a.altitude_agl, b.altitude_agl),
            altitude_asl: l(a.altitude_asl, b.altitude_asl),
            vv: l(a.vv, b.vv),
            lateral_velocity: l(a.lateral_velocity, b.lateral_velocity),
            zenith: l(a.zenith, b.zenith),
            azimuth: l(a.azimuth, b.azimuth),
            pitch_rate: match (a.pitch_rate, b.pitch_rate) {
                (Some(x), Some(y)) => Some(l(x, y)),
                _ => None,
            },
            yaw_rate: match (a.yaw_rate, b.yaw_rate) {
                (Some(x), Some(y)) => Some(l(x, y)),
                _ => None,
            },
            mass: l(a.mass, b.mass),
            thrust: l(a.thrust, b.thrust),
            drag: l(a.drag, b.drag),
            pressure: l(a.pressure, b.pressure),
            density: l(a.density, b.density),
            speed_of_sound: l(a.speed_of_sound, b.speed_of_sound),
            mach: l(a.mach, b.mach),
            gravity: l(a.gravity, b.gravity),
            reference_area: l(a.reference_area, b.reference_area),
            acc_world: a.acc_world + (b.acc_world - a.acc_world) * s,
        }
    }

    fn last_t(&self) -> f32 {
        self.rows[self.rows.len() - 1].t
    }

    /// Pad pressure altitude — the estimators' zero.
    fn pad_asl(&self) -> f32 {
        self.rows[0].altitude_asl
    }

    /// (time, pressure altitude ASL) of the true apogee.
    fn apogee(&self) -> (f32, f32) {
        let r = self
            .rows
            .iter()
            .max_by(|a, b| a.altitude_asl.total_cmp(&b.altitude_asl))
            .unwrap();
        (r.t, r.altitude_asl)
    }

    fn burnout_t(&self) -> f32 {
        // last row still producing thrust
        self.rows
            .iter()
            .filter(|r| r.thrust > 0.0)
            .map(|r| r.t)
            .fold(0.0, f32::max)
    }

    /// Time the axial specific force `(thrust - drag)/mass` crosses zero
    /// downward — the signal the estimator's burnout latch actually
    /// watches, which on a long tail-off at high Mach is NOT the motor
    /// burning out. See [`nominal_o3400_flight`].
    fn axial_zero_crossing(&self) -> f32 {
        for w in self.rows.windows(2) {
            let (a, b) = (w[0].thrust - w[0].drag, w[1].thrust - w[1].drag);
            if w[0].t > 1.0 && a > 0.0 && b <= 0.0 {
                return w[0].t + a / (a - b) * (w[1].t - w[0].t);
            }
        }
        panic!("the axial channel never went negative");
    }

    /// Time of the coast-side downward crossing of `mach`.
    fn mach_down_crossing(&self, mach: f32) -> f32 {
        let burnout = self.burnout_t();
        for w in self.rows.windows(2) {
            if w[0].t <= burnout {
                continue;
            }
            if w[0].mach > mach && w[1].mach <= mach {
                let s = (w[0].mach - mach) / (w[0].mach - w[1].mach);
                return w[0].t + s * (w[1].t - w[0].t);
            }
        }
        panic!("never crossed Mach {mach} on the coast");
    }

    /// Time the descent passes down through `agl` metres.
    fn descent_crossing_agl(&self, agl: f32) -> f32 {
        let (apogee_t, _) = self.apogee();
        for w in self.rows.windows(2) {
            if w[0].t <= apogee_t {
                continue;
            }
            let (a0, a1) = (
                w[0].altitude_asl - self.pad_asl(),
                w[1].altitude_asl - self.pad_asl(),
            );
            if a0 > agl && a1 <= agl {
                let s = (a0 - agl) / (a0 - a1);
                return w[0].t + s * (w[1].t - w[0].t);
            }
        }
        panic!("descent never reached {agl} m AGL");
    }
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

/// xorshift64*, so the runs are deterministic without a `rand` dependency.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn uniform(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Standard normal, Box-Muller.
    fn normal(&mut self) -> f32 {
        let u1 = self.uniform().max(1e-7);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Everything invented about the sensors, in one place.
///
/// The IMU noise is measured, not guessed: `VLF5/firmware`'s `imu_bench`
/// binary run against the flight board's own LSM6DSM, 57 stationary 2 s
/// windows over 114 s, median window sigma per axis. It is quite unlike the
/// Void Lake pad figures these tests first used — the accelerometer is
/// 4-10x quieter than that flight's pad (which had rail sway and a live
/// motor next to it), while gyro X is nearly twice as noisy. Static
/// pressure is still the Void Lake pad's 5.5 Pa RMS; the MS5607 bench
/// measurement lives in `hil/baro_sim.rs` as 0.36 m, which is the same
/// number in altitude form.
struct SensorModel {
    /// Accelerometer full scale, m/s^2. The LSM6DSM is configured for
    /// +-16 g in `drivers/lsm6dsm.rs`, and Osiris exceeds that in the
    /// middle of the burn — see [`clipped_accel_still_flies_the_profile`].
    accel_full_scale: f32,
    /// Per-axis, because the measured axes are genuinely not equal.
    accel_noise: Vector3<f32>,
    gyro_noise_rad_s: Vector3<f32>,
    gyro_bias_rad_s: Vector3<f32>,
    /// RMS of the pressure noise, Pa.
    pressure_noise_pa: f32,
    /// Peak static-port pressure error as a fraction of dynamic pressure.
    /// Zero disables the transonic error entirely.
    transonic_port_error: f32,
    /// Rotation from the airframe frame into the IMU chip frame.
    mount: UnitQuaternion<f32>,
    /// Seconds of pad prepended before ignition.
    pad_s: f32,
    /// Stop generating samples after this truth time.
    until_s: f32,
    /// Nominal sample interval (us). 2404 is the 416 Hz the firmware
    /// assumes; the flight board's LSM6DSM actually delivers 2342 (427 Hz),
    /// which `imu_bench` measured.
    sample_dt_us: u64,
    seed: u64,
}

impl Default for SensorModel {
    fn default() -> Self {
        Self {
            accel_full_scale: f32::INFINITY,
            accel_noise: Vector3::new(0.0147, 0.0190, 0.0359),
            gyro_noise_rad_s: Vector3::new(0.448, 0.181, 0.048) * (PI / 180.0),
            // a real, constant, uncalibrated gyro bias — around a degree
            // per second per axis, which is what the pad calibration is for
            gyro_bias_rad_s: Vector3::new(1.15, -1.93, -0.45) * (PI / 180.0),
            pressure_noise_pa: 5.5,
            mount: imu_mounting(),
            transonic_port_error: 0.0,
            sample_dt_us: 2404,
            pad_s: 60.0,
            until_s: f32::INFINITY,
            seed: 0x0517_2026_0626_0001,
        }
    }
}

/// LSM6DSM +-16 g / +-2000 dps LSBs, from `drivers/lsm6dsm.rs`.
const ACCEL_LSB: f32 = 16.0 / 32768.0 * 9.81;
const GYRO_LSB_RAD_S: f32 = (2000.0 / 32768.0) * PI / 180.0;
/// The MS5607 driver reports pressure as a whole number of Pa.
const PRESSURE_LSB_PA: f32 = 1.0;

/// The default mounting rotation between the airframe and the IMU chip.
/// Chosen to be nothing like identity so the pad self-calibration has real
/// work to do.
fn imu_mounting() -> UnitQuaternion<f32> {
    UnitQuaternion::from_euler_angles(0.31, -0.22, 2.4)
}

/// A board mounted the way a board actually gets mounted: chip +Z along the
/// airframe axis, a few degrees out. This is the WORST case for
/// accelerometer clipping, because the whole axial specific force lands on
/// one channel instead of being shared across three.
fn axis_aligned_mounting() -> UnitQuaternion<f32> {
    UnitQuaternion::from_euler_angles(0.05, -0.03, 0.7)
}

/// Roll rate (rad/s) at truth time `t`: spin-up over the burn to 1 rev/s,
/// then a slow decay. Entirely invented — OpenRocket reports zero roll for
/// this design, and a rocket that does not roll is not a useful test of a
/// gyro-integrating estimator.
fn roll_rate(t: f32, burnout_t: f32) -> f32 {
    if t <= 0.0 {
        return 0.0;
    }
    let spin_up = 1.0 - (-t / 2.0).exp();
    let decay = (-(t - burnout_t).max(0.0) / 25.0).exp();
    2.0 * PI * spin_up * decay
}

/// Airframe attitude at truth time `t`: `q * body_vector = world_vector`,
/// with body +Z along the nose.
///
/// `Rz(azimuth) * Ry(tilt) * Rz(roll)`, so body +Z lands on
/// `(sin(tilt)cos(az), sin(tilt)sin(az), cos(tilt))` — the airframe axis —
/// and the last factor spins the chip about it.
fn attitude(truth: &TruthRow, roll: f32) -> UnitQuaternion<f32> {
    let tilt = core::f32::consts::FRAC_PI_2 - truth.zenith;
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), truth.azimuth)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), tilt)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll)
}

/// Body-frame angular velocity for the same parameterisation, analytically:
/// `w_world = az_dot * Z + tilt_dot * Rz(az) Y + roll_dot * axis`, rotated
/// into the body frame. [`orientation_model_matches_openrocket`] checks
/// this against OpenRocket's own pitch and yaw rates.
fn body_rates(
    truth: &TruthRow,
    az_dot: f32,
    tilt_dot: f32,
    roll_dot: f32,
    q: &UnitQuaternion<f32>,
) -> Vector3<f32> {
    let az = truth.azimuth;
    let tilt = core::f32::consts::FRAC_PI_2 - truth.zenith;
    let axis_world = Vector3::new(
        tilt.sin() * az.cos(),
        tilt.sin() * az.sin(),
        tilt.cos(),
    );
    let w_world = Vector3::z() * az_dot
        + Vector3::new(-az.sin(), az.cos(), 0.0) * tilt_dot
        + axis_world * roll_dot;
    q.inverse_transform_vector(&w_world)
}

/// One generated sample: what the firmware's estimator loop would see,
/// plus the truth it was generated from so the test can score it.
struct Sample {
    t_us: u64,
    /// Truth time; negative while still on the pad.
    truth_t: f32,
    imu: ImuSample,
    baro_altitude_asl: f32,
    truth: TruthRow,
    /// Set when the accelerometer full scale clipped this sample.
    clipped: bool,
}

/// Generate the sensor stream: 416 Hz nominal with jitter, a quiet pad
/// segment, then the trajectory.
fn synthesize(truth: &Truth, model: &SensorModel) -> Vec<Sample> {
    let mut rng = Rng(model.seed | 1);
    let burnout_t = truth.burnout_t();
    let end_t = model.until_s.min(truth.last_t());

    // Attitude derivatives come from differentiating the truth at the
    // sample time; h is well under one OpenRocket step (5 ms on ascent).
    const H: f32 = 0.001;

    let mut samples = Vec::new();
    let mut t_us: u64 = 0;
    let mut roll = 0.0f32;
    let mut prev_truth_t = -model.pad_s;

    loop {
        let truth_t = (t_us as f32) * 1e-6 - model.pad_s;
        if truth_t > end_t {
            break;
        }

        roll += roll_rate(truth_t, burnout_t) * (truth_t - prev_truth_t);
        prev_truth_t = truth_t;

        // --- truth state, with the pad standing in before ignition ------
        let on_pad = truth_t < 0.0;
        let mut r = truth.at(truth_t.max(0.0));
        if on_pad {
            // Rail sway: a small, slow tilt oscillation, and no motion.
            r.vv = 0.0;
            r.lateral_velocity = 0.0;
            r.acc_world = Vector3::zeros();
            r.zenith += 0.03f32.to_radians() * (2.0 * PI * 0.7 * truth_t).sin();
            r.mach = 0.0;
        }

        // --- attitude and rates ----------------------------------------
        let q = attitude(&r, roll);
        let (tilt_dot, az_dot) = {
            let a = truth.at((truth_t - H).max(0.0));
            let b = truth.at((truth_t + H).max(0.0));
            if on_pad {
                // differentiate the sway analytically instead
                let w = 2.0 * PI * 0.7;
                (
                    -0.03f32.to_radians() * w * (w * truth_t).cos(),
                    0.0,
                )
            } else {
                (
                    -(b.zenith - a.zenith) / (2.0 * H),
                    (b.azimuth - a.azimuth) / (2.0 * H),
                )
            }
        };
        let w_body = body_rates(&r, az_dot, tilt_dot, roll_rate(truth_t, burnout_t), &q);

        // --- specific force --------------------------------------------
        // The accelerometer measures specific force: kinematic acceleration
        // minus gravity. On the pad that is exactly +g up.
        let sf_body = {
            let sf_world = r.acc_world + Vector3::new(0.0, 0.0, r.gravity);
            q.inverse_transform_vector(&sf_world)
        };

        // --- into the chip frame, then through the chip -----------------
        let mut acc = model.mount.inverse_transform_vector(&sf_body);
        let mut gyro = model.mount.inverse_transform_vector(&w_body);

        for k in 0..3 {
            acc[k] += rng.normal() * model.accel_noise[k];
            gyro[k] += rng.normal() * model.gyro_noise_rad_s[k] + model.gyro_bias_rad_s[k];
        }
        let mut clipped = false;
        for k in 0..3 {
            if acc[k].abs() > model.accel_full_scale {
                acc[k] = acc[k].clamp(-model.accel_full_scale, model.accel_full_scale);
                clipped = true;
            }
            acc[k] = (acc[k] / ACCEL_LSB).round() * ACCEL_LSB;
            gyro[k] = (gyro[k] / GYRO_LSB_RAD_S).round() * GYRO_LSB_RAD_S;
        }

        // --- barometer --------------------------------------------------
        let mut pressure = r.pressure;
        if model.transonic_port_error > 0.0 && !on_pad {
            let q_dyn = 0.5 * r.density * (r.mach * r.speed_of_sound).powi(2);
            pressure -= model.transonic_port_error * q_dyn * transonic_shape(r.mach);
        }
        pressure += rng.normal() * model.pressure_noise_pa;
        pressure = (pressure / PRESSURE_LSB_PA).round() * PRESSURE_LSB_PA;
        let baro_altitude_asl = calculate_isa_altitude(Pascals(pressure as f64)).0 as f32;

        samples.push(Sample {
            t_us,
            truth_t,
            imu: ImuSample { acc, gyro },
            baro_altitude_asl,
            truth: r,
            clipped,
        });

        // 416 Hz with a little jitter, the way a real sensor task delivers.
        t_us += model.sample_dt_us + (rng.uniform() * 120.0) as u64;
    }

    samples
}

/// Shape of the static-port error against Mach: nothing subsonic, rising
/// through the transonic region, held supersonic. Invented — its only job
/// is to make the barometer as dishonest through the lockout as a real
/// port is, so the tests below prove the lockout is what saves the answer.
fn transonic_shape(mach: f32) -> f32 {
    if mach < 0.7 {
        0.0
    } else if mach < 1.0 {
        (mach - 0.7) / 0.3
    } else {
        1.0
    }
}

// ---------------------------------------------------------------------------
// Replay
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Replay {
    /// (truth time, forced?) when the airbrakes filter was born
    birth: Option<(f32, bool)>,
    /// truth time the pad calibration first completed
    calibration_t: Option<f32>,
    /// truth time `burnout_detected()` first latched
    burnout_t: Option<f32>,
    burnout_unlatched: bool,
    /// continuous spans of truth time where `subsonic_by_drag()` was true
    subsonic_spans: Vec<(f32, f32)>,
    /// truth time the airbrakes half was retired (dropped by
    /// `FlightEstimators::update`)
    retired_t: Option<f32>,
    /// truth time the DEPLOYMENT half left ascent (its apogee call)
    deployment_apogee_t: Option<f32>,
    /// (truth time, PyroSelect) for every pyro command, in order
    pyros: Vec<(f32, &'static str)>,
    /// (truth time, estimated vv, true vv) while the airbrakes filter was alive
    vv_track: Vec<(f32, f32, f32)>,
    /// (truth time, estimated altitude ASL, true altitude ASL) likewise
    alt_track: Vec<(f32, f32, f32)>,
    /// (truth time, commanded extension) for every tick the gate was open
    mpc: Vec<(f32, f32)>,
    /// truth time the MPC gate first opened / last closed
    mpc_window: Option<(f32, f32)>,
    /// worst |baro altitude - true altitude| seen during the lockout
    worst_baro_error_in_lockout: f32,
    clipped_samples: usize,
    /// truth time of the FIRST sample to hit the accelerometer rail
    first_clip_t: Option<f32>,
    /// (truth time, estimated tilt, true tilt) in radians, every sample the
    /// estimator reported a tilt
    tilt_track: Vec<(f32, f32, f32)>,
}

/// Drive [`FlightEstimators`] exactly the way `armed_mode.rs` does, plus the
/// MPC on the states the gate hands out.
fn replay(samples: &[Sample], config: FlightConfig, target_apogee_asl: f32) -> Replay {
    let mpc = AirBrakesMPC::new(config.airbrakes.rocket.clone(), target_apogee_asl);
    let mut est = FlightEstimators::new(config);
    let mut out = Replay::default();
    let mut span_start: Option<f32> = None;

    for s in samples {
        let (pyro, _log) = est.update(s.t_us, Some(&s.imu), s.baro_altitude_asl);
        let t = s.truth_t;

        if s.clipped {
            out.clipped_samples += 1;
            if out.first_clip_t.is_none() {
                out.first_clip_t = Some(t);
            }
        }
        if let Some(p) = pyro {
            out.pyros.push((t, pyro_name(p)));
        }

        if out.retired_t.is_none()
            && est.airbrakes_estimator().is_none()
            && !out.alt_track.is_empty()
        {
            out.retired_t = Some(t);
        }
        if let Some(ab) = est.airbrakes_estimator() {
            if out.calibration_t.is_none() && ab.calibration_complete() {
                out.calibration_t = Some(t);
            }
            match (ab.burnout_detected(), out.burnout_t) {
                (true, None) => out.burnout_t = Some(t),
                (false, Some(_)) => out.burnout_unlatched = true,
                _ => {}
            }
            match (ab.subsonic_by_drag(), span_start) {
                (Some(true), None) => span_start = Some(t),
                (Some(true), Some(_)) => {}
                (_, Some(start)) => {
                    out.subsonic_spans.push((start, t));
                    span_start = None;
                }
                _ => {}
            }
            if out.birth.is_none()
                && let Some((born_us, forced)) = ab.birth()
            {
                out.birth = Some(((born_us as f32) * 1e-6 - samples[0].truth_t.abs(), forced));
            }
            if !ab.airbrakes_enabled() && out.birth.is_none() && t > 0.0 {
                let err = (s.baro_altitude_asl - s.truth.altitude_asl).abs();
                out.worst_baro_error_in_lockout = out.worst_baro_error_in_lockout.max(err);
            }
            if let Some(tilt) = ab.tilt() {
                out.tilt_track.push((
                    t,
                    tilt,
                    core::f32::consts::FRAC_PI_2 - s.truth.zenith,
                ));
            }
            if let (Some(v), Some(a)) = (ab.velocity(), ab.altitude_asl()) {
                out.vv_track.push((t, v.y, s.truth.vv));
                out.alt_track.push((t, a, s.truth.altitude_asl));
            }
        }

        if out.deployment_apogee_t.is_none()
            && !matches!(
                est.state(),
                crate::RocketState::OnPad
                    | crate::RocketState::Ascent { .. }
                    | crate::RocketState::MachLockout { .. }
            )
        {
            out.deployment_apogee_t = Some(t);
        }

        if let Some(states) = est.airbrakes_mpc_states() {
            let sol = mpc.update(states.altitude_asl, states.velocity);
            out.mpc.push((t, sol.extension_percentage));
            out.mpc_window = Some(match out.mpc_window {
                None => (t, t),
                Some((a, _)) => (a, t),
            });
        }
    }

    out
}

fn pyro_name(p: firmware_common_new::vlp::packets::fire_pyro::PyroSelect) -> &'static str {
    use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
    match p {
        PyroSelect::PyroDrogue => "drogue",
        PyroSelect::PyroMain => "main",
    }
}

impl Replay {
    /// The airbrakes filter's own apogee: the peak of the altitude it
    /// reported, and when. This is the number worth scoring, because the
    /// filter is retired at zero vertical velocity and the peak it reached
    /// right before that IS its apogee estimate.
    ///
    /// There has never been anything else to score it against here. The
    /// estimator's own 0.5 s apogee latch — deleted on 2026-08-17 — needed
    /// 0.5 s below 1 m/s and the airbrakes half is dropped at 0 m/s, so it
    /// never once fired in a composed flight; this field recorded `None` on
    /// every simulation in this file.
    fn estimated_apogee(&self) -> Option<(f32, f32)> {
        self.alt_track
            .iter()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(t, alt, _)| (*t, *alt))
    }
}

/// Mean |estimate - truth| over a truth-time window.
fn mean_error(track: &[(f32, f32, f32)], from: f32, to: f32) -> (f32, usize) {
    let mut sum = 0.0;
    let mut n = 0;
    for (t, est, truth) in track {
        if *t >= from && *t < to {
            sum += (est - truth).abs();
            n += 1;
        }
    }
    (if n > 0 { sum / n as f32 } else { f32::NAN }, n)
}

// ===========================================================================
// 1. The model itself — if these fail, nothing below means anything
// ===========================================================================

/// The attitude parameterisation is checked against data it was not built
/// from: OpenRocket reports body pitch and yaw rates independently, and the
/// non-roll part of the body rate this file synthesises must equal them.
///
/// Compared on the raw rows, never on interpolated ones. OpenRocket's pitch
/// and yaw columns flip sign together where its body-axis decomposition
/// changes branch (visible around ignition+20 s on the O3400) — the
/// magnitude is continuous across the flip but the components are not, so
/// interpolating them would compare against a number that never existed.
#[test]
fn orientation_model_matches_openrocket() {
    init_logger();
    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let rows = &truth.rows;

        let mut worst = 0.0f32;
        let mut worst_t = 0.0f32;
        let mut n = 0;
        let mut gaps = 0;
        for i in 1..rows.len() - 1 {
            let r = rows[i];
            if r.t < 0.5 {
                continue;
            }
            let (Some(pitch), Some(yaw)) = (r.pitch_rate, r.yaw_rate) else {
                break;
            };
            // OpenRocket emits an occasional row with both rates exactly
            // zero while the neighbours either side agree to five decimals
            // — a gap in its own output, not a moment the airframe stopped
            // rotating. Skipping it, and counting how many there were.
            if pitch == 0.0 && yaw == 0.0 {
                gaps += 1;
                continue;
            }
            let dt = rows[i + 1].t - rows[i - 1].t;
            if dt <= 0.0 {
                continue;
            }
            let tilt_dot = -(rows[i + 1].zenith - rows[i - 1].zenith) / dt;
            let az_dot = (rows[i + 1].azimuth - rows[i - 1].azimuth) / dt;
            // roll excluded on both sides: OpenRocket's roll rate is zero
            // here, and the roll this file adds is synthetic
            let q = attitude(&r, 0.0);
            let w = body_rates(&r, az_dot, tilt_dot, 0.0, &q);

            let mine = (w.x * w.x + w.y * w.y).sqrt();
            let theirs = (pitch * pitch + yaw * yaw).sqrt();
            // relative, with an absolute floor: the finite difference of a
            // stored column cannot resolve better than the integrator wrote
            let err = (mine - theirs).abs() / (0.15 * theirs + 0.005);
            if err > worst {
                worst = err;
                worst_t = r.t;
            }
            n += 1;
        }
        eprintln!(
            "{path}: body rates within {worst:.2} of tolerance (worst at t={worst_t:.2}s, \
             {n} rows, {gaps} skipped as OpenRocket output gaps)"
        );
        assert!(n > 5000, "{path}: only {n} rows compared");
        assert!(
            gaps * 100 < n,
            "{path}: {gaps} of {n} rows had no body rates — that is not a gap, \
             that is the column being unusable"
        );
        assert!(
            worst < 1.0,
            "{path}: synthesised body rates disagree with OpenRocket's at t={worst_t}s"
        );
    }
}

/// The synthesised specific force is checked against data it was not built
/// from either. The accelerometer here comes from differentiating
/// OpenRocket's velocity columns; an accelerometer measures specific force,
/// which in free flight is the whole thrust-plus-aerodynamic force over
/// mass. Projected on the airframe axis, that must equal
/// `(thrust - drag) / mass` from OpenRocket's own force columns — signed,
/// so the comparison stays honest straight through the tail-off where the
/// axial channel crosses zero (which is the crossing the burnout latch
/// lives on).
#[test]
fn sensor_model_matches_openrocket_forces() {
    init_logger();
    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let rows = &truth.rows;

        let mut worst = 0.0f32;
        let mut worst_t = 0.0f32;
        let mut n = 0;
        for r in rows.iter() {
            // skip the ignition transient, and stop before the recovery
            // deployment turns this into a parachute problem
            if r.t < 0.3 || r.t > 37.0 {
                continue;
            }
            let sf = r.acc_world + Vector3::new(0.0, 0.0, r.gravity);
            let tilt = core::f32::consts::FRAC_PI_2 - r.zenith;
            let axis = Vector3::new(
                tilt.sin() * r.azimuth.cos(),
                tilt.sin() * r.azimuth.sin(),
                tilt.cos(),
            );
            let expected = (r.thrust - r.drag) / r.mass;
            // 5% relative, with a 0.3 m/s^2 floor: late in the coast the
            // whole signal is under 1 m/s^2 and a relative test there is
            // measuring the file's own round-off
            let err = (sf.dot(&axis) - expected).abs() / (0.05 * expected.abs() + 0.3);
            if err > worst {
                worst = err;
                worst_t = r.t;
            }
            n += 1;
        }
        eprintln!(
            "{path}: axial specific force within {worst:.2} of tolerance vs \
             OpenRocket's thrust and drag (worst at t={worst_t:.2}s, {n} rows)"
        );
        assert!(n > 3000, "{path}: only {n} rows compared");
        assert!(
            worst < 1.0,
            "{path}: synthesised specific force disagrees with OpenRocket's forces \
             at t={worst_t}s"
        );
    }
}

/// The reference area in the flight config must be the one OpenRocket used
/// to produce these trajectories — the Cd table is only meaningful paired
/// with it.
#[test]
fn config_matches_the_simulated_airframe() {
    init_logger();
    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let sim_area = truth.rows[0].reference_area;
        let cfg_area = osiris_rocket().reference_area;
        assert!(
            (sim_area - cfg_area).abs() / cfg_area < 1e-3,
            "{path}: reference area {sim_area} vs config {cfg_area}"
        );
        // burnout mass: OpenRocket's post-burnout mass vs the config
        let m = truth.at(truth.burnout_t() + 1.0).mass;
        assert!(
            (m - osiris_rocket().burnout_mass).abs() < 0.25,
            "{path}: burnout mass {m} vs config {}",
            osiris_rocket().burnout_mass
        );
    }
}

// ===========================================================================
// 2. The config's timing constants against every simulation in the document
// ===========================================================================

/// The three timers in `FLIGHT_CONFIG` are claims about the trajectory.
/// Check them against both motors, to the rules their doc comments state:
///
/// * `earliest_subsonic_after_ignition_us` must not be later than the
///   earliest true Mach-0.8 crossing (erring late only costs control
///   window; erring early is unsafe, so it must also not be so early that
///   the check could be consulted while supersonic — that is the >= side).
/// * `force_birth_after_ignition_us` must be past the latest true crossing
///   and at least 5 s before the earliest apogee.
/// * `mach_lockout_duration_us` must be past the true Mach-0.75 crossing
///   and at least 5 s before apogee.
#[test]
fn mach_lockout_timers_bracket_every_simulation() {
    init_logger();
    let cfg = osiris_config();
    let ml = cfg.airbrakes.mach_lockout.clone().unwrap();
    let t_early = ml.earliest_subsonic_after_ignition_us as f32 * 1e-6;
    let t_force = ml.force_birth_after_ignition_us as f32 * 1e-6;
    let t_freeze = cfg.profile.mach_lockout_duration_us.unwrap() as f32 * 1e-6;

    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let m08 = truth.mach_down_crossing(0.8);
        let m075 = truth.mach_down_crossing(0.75);
        let (apogee_t, apogee_asl) = truth.apogee();
        eprintln!(
            "{path}: Mach 0.8 at {m08:.2}s ({:.0} m ASL), Mach 0.75 at {m075:.2}s, \
             apogee {apogee_asl:.0} m pressure-ASL at {apogee_t:.2}s",
            truth.at(m08).altitude_asl
        );

        // Never able to CONCLUDE while genuinely supersonic. The check
        // must hold continuously for `SUBSONIC_SUSTAIN_S` (1 s) before it
        // approves a birth, so the earliest possible birth is the gate
        // opening plus one second — that is the number that has to clear
        // the true crossing, not the gate itself.
        const SUBSONIC_SUSTAIN_S: f32 = 1.0;
        assert!(
            t_early + SUBSONIC_SUSTAIN_S >= m08,
            "{path}: the check could approve a birth at {}s, while the airframe \
             is still above Mach 0.8 until {m08}s",
            t_early + SUBSONIC_SUSTAIN_S
        );
        // Erring late is safe but costs control window; say how much.
        if t_early > m08 {
            eprintln!(
                "{path}:   note — the O3400-sized floor holds the check shut for \
                 {:.2}s after this motor is already subsonic",
                t_early - m08
            );
        }
        // The atmosphere the drag check inverts with is a configured
        // constant, so a motor that crosses somewhere other than the
        // configured altitude has its airspeed read wrong — and the only
        // direction that matters is reading LOW, which opens the lockout
        // early. Fold both altitude-dependent terms (density and the speed
        // of sound the threshold is scaled by) into the one number that
        // decides: the TRUE Mach at which the check actually votes.
        //
        //   fires when  v_true * sqrt(rho(h_t)/rho(h_c))  <  M_cfg * a(h_c)
        //   i.e. when   M_true  <  M_cfg * a(h_c)/a(h_t) * sqrt(rho(h_c)/rho(h_t))
        //
        // For the motor the constant was taken from this is exactly `M_cfg`.
        // For any motor crossing LOWER, both factors fall below 1 and the
        // check votes conservatively — which is the whole reason the
        // constant is set from the highest crossing rather than the mean.
        let h_c = ml.subsonic_crossing_altitude_asl;
        let h_t = truth.at(m08).altitude_asl;
        let effective_mach = cfg.airbrakes.max_open_mach
            * (approximate_speed_of_sound(h_c) / approximate_speed_of_sound(h_t))
            * libm::sqrtf(approximate_air_density(h_c) / approximate_air_density(h_t));
        eprintln!(
            "{path}:   crosses at {h_t:.0} m against the configured {h_c:.0} m, so the \
             drag check really votes at Mach {effective_mach:.3}"
        );
        assert!(
            effective_mach <= cfg.airbrakes.max_open_mach,
            "{path}: the configured crossing altitude {h_c:.0} m sits BELOW this \
             motor's real crossing at {h_t:.0} m, so the check votes at Mach \
             {effective_mach:.3} — above the configured {:.2}, in flow the airframe \
             is not qualified for",
            cfg.airbrakes.max_open_mach
        );

        // The timeout is a backstop, not the normal path.
        assert!(
            t_force > m08,
            "{path}: forced birth at {t_force}s beats the real crossing at {m08}s"
        );
        assert!(
            apogee_t - t_force >= 5.0,
            "{path}: forced birth at {t_force}s leaves only {:.1}s before apogee",
            apogee_t - t_force
        );
        // Deployment-half lockout.
        assert!(
            t_freeze > m075,
            "{path}: baro unfrozen at {t_freeze}s, still above Mach 0.75 until {m075}s"
        );
        assert!(
            apogee_t - t_freeze >= 5.0,
            "{path}: baro unfrozen only {:.1}s before apogee",
            apogee_t - t_freeze
        );

        // Drogue floor must be well under the apogee it guards.
        let DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: floor,
            ..
        } = cfg.profile.deployment
        else {
            panic!("the flight config is no longer a dual deployment");
        };
        assert!(
            apogee_asl - truth.pad_asl() > floor * 2.0,
            "{path}: apogee {:.0} m AGL vs drogue floor {floor}", apogee_asl - truth.pad_asl()
        );
    }
}

// ===========================================================================
// 3. The nominal flight
// ===========================================================================

/// Osiris on the O3400, clean sensors, honest barometer: the whole chain
/// from pad calibration to apogee, scored against the OpenRocket truth.
#[test]
fn nominal_o3400_flight() {
    init_logger();
    let truth = Truth::load(O3400_CSV);
    let (apogee_t, apogee_asl) = truth.apogee();
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: apogee_t + 15.0,
            ..Default::default()
        },
    );
    eprintln!("nominal: {} samples", samples.len());

    // Target 150 m below the natural apogee. The brakes on this airframe
    // are worth ~260 m from the state the filter is born in (see
    // `airbrakes_authority_and_mpc_convergence`), so anything much deeper
    // than this is not a control problem, it is a saturated rail.
    let target_asl = apogee_asl - 150.0;
    let r = replay(&samples, osiris_config(), target_asl);

    // --- pad ---------------------------------------------------------
    let cal = r.calibration_t.expect("pad calibration never completed");
    eprintln!("nominal: calibration complete at ignition{cal:+.1}s");
    assert!(cal < 0.0, "calibration only completed after ignition ({cal}s)");

    // --- burnout latch ------------------------------------------------
    //
    // The latch watches the SIGN of the axial specific force, and on this
    // flight that sign flips well before the motor is out: at Mach 1.9 the
    // airframe is eating 1200 N of drag, and the O3400's tail-off drops
    // below that a full second before it stops burning. So the latch fires
    // during tail-off, with several hundred newtons still on the case.
    //
    // That is the unsafe direction for the drag check — residual thrust
    // cancels drag and the check would invert an unrealistically low
    // deceleration into an unrealistically low speed. What stops it
    // mattering is the OTHER guard: `earliest_subsonic_after_ignition_us`
    // holds the check shut until 17.5 s, by which point the motor has been
    // out for eleven seconds. Both halves of that are asserted here,
    // because the safety argument needs both.
    let burnout = truth.burnout_t();
    let crossing = truth.axial_zero_crossing();
    let latched = r.burnout_t.expect("burnout never latched");
    let residual_thrust = truth.at(latched).thrust;
    eprintln!(
        "nominal: axial channel goes negative at {crossing:.2}s, motor actually out \
         at {burnout:.2}s; latch at {latched:.2}s with {residual_thrust:.0} N still \
         burning"
    );
    assert!(!r.burnout_unlatched, "burnout latch went back to false");
    assert!(
        latched > crossing && latched < crossing + 0.6,
        "latch at {latched}s does not follow the axial sign crossing at {crossing}s"
    );
    // The guard that makes the early latch harmless.
    let t_early = osiris_config()
        .airbrakes
        .mach_lockout
        .unwrap()
        .earliest_subsonic_after_ignition_us as f32
        * 1e-6;
    assert!(
        t_early > burnout + 5.0,
        "the drag check may open at {t_early}s, only {:.1}s after the motor stops \
         burning at {burnout}s — the latch fires during tail-off, so this floor is \
         the only thing keeping residual thrust out of the drag inversion",
        t_early - burnout
    );

    // --- lockout exit --------------------------------------------------
    let (born, forced) = r.birth.expect("airbrakes filter never born");
    let m08 = truth.mach_down_crossing(0.8);
    eprintln!(
        "nominal: born {born:.2}s (forced: {forced}); true Mach 0.8 at {m08:.2}s; \
         drag-check spans {:?}",
        r.subsonic_spans
    );
    assert!(
        !forced,
        "the drag check never fired — birth fell through to the T_max timeout"
    );
    assert!(
        born > m08,
        "filter born at {born}s, while the airframe was still above Mach 0.8 ({m08}s)"
    );
    assert!(
        born < m08 + 3.0,
        "filter born {:.1}s after the crossing — control window wasted",
        born - m08
    );
    // The check must never once read subsonic while genuinely supersonic.
    for (start, end) in &r.subsonic_spans {
        assert!(
            *start >= m08,
            "drag check read subsonic at {start}s..{end}s, before the true \
             Mach 0.8 crossing at {m08}s"
        );
    }
    // The filter is born after the motor is out, always.
    assert!(born > latched, "filter born at {born}s, before the burnout latch");

    // --- coast accuracy vs truth ---------------------------------------
    let (vv_err, n) = mean_error(&r.vv_track, born + 2.0, apogee_t - 2.0);
    let (alt_err, _) = mean_error(&r.alt_track, born + 2.0, apogee_t - 2.0);
    eprintln!("nominal: coast |vv err| {vv_err:.2} m/s, |alt err| {alt_err:.2} m over {n} samples");
    assert!(n > 2000, "filter was not alive through the coast ({n} samples)");
    assert!(vv_err < 8.0, "coast vertical velocity error {vv_err} m/s");
    assert!(alt_err < 25.0, "coast altitude error {alt_err} m");

    // --- apogee ---------------------------------------------------------
    let (ab_t, ab_alt) = r.estimated_apogee().expect("airbrakes filter reported no altitude");
    let dep_t = r.deployment_apogee_t.expect("deployment half never called apogee");
    eprintln!(
        "nominal: true apogee {:.0} m pressure-ASL at {apogee_t:.2}s | airbrakes \
         {:+.2}s / {:+.0}m | deployment {:+.2}s",
        apogee_asl,
        ab_t - apogee_t,
        ab_alt - (apogee_asl),
        dep_t - apogee_t
    );
    assert!((ab_t - apogee_t).abs() < 3.0, "airbrakes apogee time error");
    assert!(
        (ab_alt - (apogee_asl)).abs() < 80.0,
        "airbrakes apogee altitude error"
    );
    assert!(
        (-1.0..4.0).contains(&(dep_t - apogee_t)),
        "deployment apogee call {:+.2}s off truth",
        dep_t - apogee_t
    );

    // --- the MPC window --------------------------------------------------
    let (open, close) = r.mpc_window.expect("the airbrakes gate never opened");
    eprintln!(
        "nominal: MPC gate open {open:.2}s..{close:.2}s ({:.1}s of control window, \
         {} ticks), last command {:.0}%",
        close - open,
        r.mpc.len(),
        r.mpc.last().unwrap().1 * 100.0
    );
    assert!(
        close - open > 15.0,
        "only {:.1}s of control window",
        close - open
    );
    // The gate must not open before the filter is born, and must be shut
    // again by apogee.
    assert!(open >= born, "gate opened at {open}s, before birth at {born}s");
    assert!(close <= apogee_t + 0.5, "gate still open at {close}s, past apogee");
    // Command profile. The MPC predicts apogee for "brake for one 0.1 s
    // tick, then coast stowed", so one tick is worth a couple of metres and
    // the solution is necessarily bang-bang: hold full extension until the
    // stowed-from-here prediction has fallen to the target, then modulate
    // off. What has to be true is that it does both.
    let samples_at: Vec<String> = [0.0f32, 2.0, 5.0, 10.0, 15.0, 19.0]
        .iter()
        .filter_map(|dt| {
            let want = open + dt;
            r.mpc
                .iter()
                .find(|(t, _)| *t >= want)
                .map(|(t, e)| format!("{:.0}s:{:.0}%", t - open, e * 100.0))
        })
        .collect();
    eprintln!("nominal: extension after birth — {}", samples_at.join("  "));
    let saturated = r.mpc.iter().filter(|(_, e)| *e > 0.98).count();
    assert!(
        saturated > r.mpc.len() / 10,
        "the MPC never commanded full extension ({saturated}/{} ticks) even though \
         the target is below the stowed apogee",
        r.mpc.len()
    );
    assert!(
        r.mpc.iter().any(|(_, e)| *e < 0.98),
        "the MPC sat on the full-extension rail for the entire window — it never \
         caught the target"
    );
}

/// The same flight with the barometer lying the way a real static port on a
/// Mach 1.9 airframe lies. The lockout is the only thing standing between
/// that and the filter, so the answers must barely move.
#[test]
fn transonic_static_port_error_is_absorbed_by_the_lockout() {
    init_logger();
    let truth = Truth::load(O3400_CSV);
    let (apogee_t, apogee_asl) = truth.apogee();
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: apogee_t + 15.0,
            transonic_port_error: 0.08,
            ..Default::default()
        },
    );
    let r = replay(&samples, osiris_config(), apogee_asl - 150.0);

    eprintln!(
        "port error: worst baro altitude error during lockout {:.0} m",
        r.worst_baro_error_in_lockout
    );
    // The injected error has to actually be catastrophic, or this test
    // proves nothing.
    assert!(
        r.worst_baro_error_in_lockout > 400.0,
        "the injected static-port error was too mild to test anything ({:.0} m)",
        r.worst_baro_error_in_lockout
    );

    let (born, forced) = r.birth.expect("filter never born");
    let m08 = truth.mach_down_crossing(0.8);
    assert!(!forced, "birth fell through to the T_max timeout");
    assert!(
        born > m08,
        "born at {born}s while still supersonic ({m08}s) — the drag check was \
         poisoned by the baro"
    );
    for (start, end) in &r.subsonic_spans {
        assert!(*start >= m08, "drag check read subsonic at {start}s..{end}s");
    }

    let (ab_t, ab_alt) = r.estimated_apogee().expect("airbrakes filter reported no altitude");
    let err_m = ab_alt - (apogee_asl);
    eprintln!(
        "port error: born {born:.2}s, apogee {:+.2}s / {err_m:+.0}m vs truth",
        ab_t - apogee_t
    );
    assert!((ab_t - apogee_t).abs() < 3.0);
    assert!(err_m.abs() < 80.0);

    let (vv_err, n) = mean_error(&r.vv_track, born + 2.0, apogee_t - 2.0);
    eprintln!("port error: coast |vv err| {vv_err:.2} m/s over {n} samples");
    assert!(vv_err < 8.0, "coast vv error {vv_err} m/s");
}

/// Osiris pulls 17.6 g of specific force in the middle of the burn and the
/// LSM6DSM is configured for +-16 g, so the accelerometer WILL clip — this
/// is the Void Lake failure, on an airframe where it is guaranteed rather
/// than incidental.
///
/// The dead reckoner therefore under-integrates and its velocity reads low.
/// Reading low is the dangerous direction: a low speed estimate is what
/// opens the lockout. The properties that must survive are the safety ones —
/// the lockout must not open early, and the baro must pull the filter back
/// after birth.
#[test]
fn clipped_accel_still_flies_the_profile() {
    init_logger();
    let truth = Truth::load(O3400_CSV);
    let (apogee_t, apogee_asl) = truth.apogee();
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: apogee_t + 15.0,
            accel_full_scale: 16.0 * 9.81,
            // worst case on purpose: the axial load on one channel
            mount: axis_aligned_mounting(),
            ..Default::default()
        },
    );
    let r = replay(&samples, osiris_config(), apogee_asl - 150.0);

    eprintln!(
        "clipped: {} of {} samples hit the +-16 g rail",
        r.clipped_samples,
        samples.len()
    );
    assert!(
        r.clipped_samples > 400,
        "the accelerometer never clipped — this test is not exercising anything"
    );

    let (born, forced) = r.birth.expect("filter never born");
    let m08 = truth.mach_down_crossing(0.8);
    eprintln!("clipped: born {born:.2}s (forced: {forced}), true Mach 0.8 at {m08:.2}s");

    // The one number that says how much the clipping actually cost: the
    // velocity the filter is BORN with is the dead-reckoned one, integrated
    // straight through the clipped burn. Everything before birth is thrown
    // away, so this is the entire leak.
    let (bt, b_est, b_truth) = r.vv_track[0];
    eprintln!(
        "clipped: born with vv {b_est:.1} m/s against a true {b_truth:.1} m/s at \
         t={bt:.2}s — dead-reckoning deficit {:+.1} m/s ({:+.1}%)",
        b_est - b_truth,
        (b_est - b_truth) / b_truth * 100.0
    );
    assert!(
        born > m08,
        "clipping opened the lockout at {born}s, while still supersonic ({m08}s)"
    );
    // How early the check is allowed to SPEAK, which is not the same as how
    // early it is allowed to CONCLUDE. The design rule — stated in
    // `mach_lockout_timers_bracket_every_simulation`, which checks the
    // config against exactly this — is that the check must hold for
    // `SUBSONIC_SUSTAIN_S` (1 s) before it approves a birth, so the number
    // that has to clear the true crossing is the gate opening PLUS one
    // second, not the gate itself. `born > m08` above is that property, and
    // it passes here with ~1 s to spare.
    //
    // This bounds the other end: a drag model wrong enough to vote subsonic
    // seconds early would still eat the sustain and has to fail. Under
    // clipping the check does vote a few tens of milliseconds before the
    // crossing (measured -35 ms) — clipping drags the dead-reckoned
    // altitude low, which reads the density high and the inverted airspeed
    // low — and that is the honest worst case, well inside the sustain.
    const CHECK_MAY_LEAD_S: f32 = 0.25;
    for (start, end) in &r.subsonic_spans {
        assert!(
            *start >= m08 - CHECK_MAY_LEAD_S,
            "drag check read subsonic at {start}s..{end}s under clipping, \
             {:.3}s before the true Mach 0.8 crossing at {m08}s — more than \
             the {CHECK_MAY_LEAD_S}s the 1 s sustain is allowed to absorb",
            m08 - *start
        );
    }

    // The deficit is real but it does not live long: the filter is born
    // with a deliberately large velocity variance, so the barometer
    // dominates immediately. Measured against the same run with the same
    // mounting and no clipping, which is born within 0.3 m/s of truth.
    let (rt, r_est, r_truth) = *r
        .vv_track
        .iter()
        .find(|(t, _, _)| *t >= born + 1.0)
        .expect("filter died within a second of birth");
    eprintln!(
        "clipped: one second after birth (t={rt:.2}s) the deficit is {:+.1} m/s",
        r_est - r_truth
    );
    assert!(
        (r_est - r_truth).abs() < 5.0,
        "a second after birth the velocity is still {:+.1} m/s out — the baro is \
         not pulling the clipped dead reckoning back",
        r_est - r_truth
    );

    // After birth the baro is trusted again, so the wrong dead-reckoned
    // velocity must be pulled back within a few seconds.
    let (vv_err, n) = mean_error(&r.vv_track, born + 5.0, apogee_t - 2.0);
    eprintln!("clipped: coast |vv err| {vv_err:.2} m/s over {n} samples (from birth+5s)");
    assert!(n > 1000);
    assert!(vv_err < 10.0, "velocity never recovered: {vv_err} m/s");

    let (ab_t, ab_alt) = r.estimated_apogee().expect("airbrakes filter reported no altitude");
    eprintln!(
        "clipped: apogee {:+.2}s / {:+.0}m vs truth",
        ab_t - apogee_t,
        ab_alt - (apogee_asl)
    );
    assert!((ab_t - apogee_t).abs() < 3.5);
    assert!((ab_alt - (apogee_asl)).abs() < 100.0);
}

/// The backup motor. It is 200 m/s slower and goes subsonic 1.9 s earlier
/// than the O3400 the timers were sized from, so the check opens after the
/// airframe is already subsonic — safe, but it costs control window, and
/// this test is what says how much.
#[test]
fn backup_motor_n2900_flight() {
    init_logger();
    let truth = Truth::load(N2900_CSV);
    let (apogee_t, apogee_asl) = truth.apogee();
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: apogee_t + 15.0,
            ..Default::default()
        },
    );
    let r = replay(&samples, osiris_config(), apogee_asl - 150.0);

    let (born, forced) = r.birth.expect("filter never born");
    let m08 = truth.mach_down_crossing(0.8);
    let t_early = osiris_config()
        .airbrakes
        .mach_lockout
        .unwrap()
        .earliest_subsonic_after_ignition_us as f32
        * 1e-6;
    eprintln!(
        "n2900: true Mach 0.8 at {m08:.2}s, check may not open before {t_early:.2}s, \
         born {born:.2}s (forced: {forced}) — {:.1}s of control window lost to the \
         O3400-sized timer",
        born - m08 - 1.0
    );
    assert!(!forced);
    assert!(born > m08);

    let (ab_t, ab_alt) = r.estimated_apogee().expect("airbrakes filter reported no altitude");
    eprintln!(
        "n2900: apogee {:+.2}s / {:+.0}m vs truth ({apogee_asl:.0} m pressure-ASL at {apogee_t:.2}s)",
        ab_t - apogee_t,
        ab_alt - (apogee_asl)
    );
    assert!((ab_t - apogee_t).abs() < 3.0);
    assert!((ab_alt - (apogee_asl)).abs() < 80.0);

    let (open, close) = r.mpc_window.expect("the airbrakes gate never opened");
    eprintln!("n2900: MPC gate open {open:.2}s..{close:.2}s ({:.1}s)", close - open);
    assert!(
        close - open > 15.0,
        "only {:.1}s of control window on the backup motor",
        close - open
    );
}

// ===========================================================================
// 4. Recovery — the half that fires the pyros
// ===========================================================================

/// The full flight to below the main deployment altitude: drogue one second
/// after the apogee call, main at 1500 ft AGL, each exactly once and in
/// order. This is the half the 1 s drogue delay in `FLIGHT_CONFIG` lives in.
#[test]
fn deployment_fires_drogue_then_main() {
    init_logger();
    let truth = Truth::load(O3400_CSV);
    let (apogee_t, _) = truth.apogee();
    let main_agl = 457.2f32;
    let main_t = truth.descent_crossing_agl(main_agl);

    let samples = synthesize(
        &truth,
        &SensorModel {
            // just past the main crossing; the descent is 5 minutes long
            until_s: main_t + 20.0,
            ..Default::default()
        },
    );
    eprintln!("deployment: {} samples to {:.0}s", samples.len(), main_t + 20.0);
    let r = replay(&samples, osiris_config(), 0.0);

    eprintln!("deployment: pyros {:?}", r.pyros);
    assert_eq!(r.pyros.len(), 2, "expected exactly a drogue and a main");
    assert_eq!(r.pyros[0].1, "drogue");
    assert_eq!(r.pyros[1].1, "main");

    // Drogue: the configured 1 s after the apogee call, which is itself
    // allowed to lag the true apogee a little.
    let dep_t = r.deployment_apogee_t.expect("no apogee call");
    let drogue_lag = r.pyros[0].0 - dep_t;
    eprintln!(
        "deployment: true apogee {apogee_t:.2}s, apogee call {dep_t:+.2}s, \
         drogue at call+{drogue_lag:.2}s ({:+.2}s vs true apogee)",
        r.pyros[0].0 - apogee_t
    );
    assert!(
        (0.9..1.3).contains(&drogue_lag),
        "drogue fired {drogue_lag}s after the apogee call, config says 1.0s"
    );

    // Main: at the configured altitude, judged against the truth trajectory.
    let main_err = r.pyros[1].0 - main_t;
    eprintln!(
        "deployment: true {main_agl} m AGL crossing at {main_t:.1}s, main fired \
         at {:.1}s ({main_err:+.2}s)",
        r.pyros[1].0
    );
    assert!(
        main_err.abs() < 2.0,
        "main fired {main_err:+.2}s off the {main_agl} m AGL crossing"
    );
}

// ===========================================================================
// 5. Airbrakes authority — can this airframe reach a target at all?
// ===========================================================================

/// Not a regression test so much as the number the flight config implies:
/// starting from the state the estimator is actually born in, how far down
/// can the brakes pull apogee at full extension, and does the MPC's
/// bisection converge on a target inside that range?
///
/// Uses the crate's own 2D dynamics, i.e. the MPC's own model — so it
/// answers "is the commanded solution self-consistent and is the target
/// reachable per the Cd table", not "will the real rocket land there".
#[test]
fn airbrakes_authority_and_mpc_convergence() {
    init_logger();
    use crate::controller::rocket_dynamics::calculate_state_derivatives;
    use crate::controller::{Derivative, State};

    let truth = Truth::load(O3400_CSV);
    let rocket = osiris_rocket();
    let (apogee_t, apogee_asl) = truth.apogee();

    // The state the filter is born in, from the real replay.
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: apogee_t + 15.0,
            ..Default::default()
        },
    );
    let r = replay(&samples, osiris_config(), 0.0);
    let (born, _) = r.birth.unwrap();
    let b = truth.at(born + 0.05);
    let start = State {
        altitude_asl: b.altitude_asl,
        velocity: Vector2::new(b.lateral_velocity, b.vv),
    };
    eprintln!(
        "authority: born at {born:.2}s, {:.0} m ASL, v = ({:.0}, {:.0}) m/s",
        start.altitude_asl, start.velocity.x, start.velocity.y
    );

    // Fixed-extension coast to apogee, on the crate's own dynamics.
    let coast_to_apogee = |drag_percentage: f32| {
        let mut s = State {
            altitude_asl: start.altitude_asl,
            velocity: start.velocity,
        };
        let dt = 0.02f32;
        while s.velocity.y > 0.0 {
            let Derivative(k) = calculate_state_derivatives(drag_percentage, &s, &rocket);
            s = State {
                altitude_asl: s.altitude_asl + k.altitude_asl * dt,
                velocity: s.velocity + k.velocity * dt,
            };
        }
        s.altitude_asl
    };

    let stowed = coast_to_apogee(-1.0);
    let full = coast_to_apogee(1.0);
    let truth_asl = apogee_asl;
    eprintln!(
        "authority: stowed coast reaches {stowed:.0} m ASL (OpenRocket truth {truth_asl:.0}), \
         full extension {full:.0} m ASL — {:.0} m of authority",
        stowed - full
    );

    // Sanity: the crate's stowed coast must land near OpenRocket's apogee,
    // or its Cd/mass/area do not describe this rocket.
    assert!(
        (stowed - truth_asl).abs() < 350.0,
        "stowed coast from the birth state reaches {stowed:.0} m ASL but \
         OpenRocket says {truth_asl:.0} m — the airframe parameters disagree \
         with the simulation"
    );
    // The authority is small: the coast from the birth state is over 2 km,
    // and full extension buys back only about a tenth of it. That is the
    // number that decides what target apogee is even askable for.
    let coast = stowed - start.altitude_asl;
    eprintln!(
        "authority: the coast is {coast:.0} m, so the brakes are worth {:.1}% of it",
        (stowed - full) / coast * 100.0
    );
    assert!(
        stowed - full > 150.0,
        "the brakes are worth only {:.0} m of apogee — check the Cd table",
        stowed - full
    );

    // Closed loop: re-solve the MPC every 0.1 s and fly the commanded
    // extension, for a target inside the authority band.
    for frac in [0.25f32, 0.5, 0.75] {
        let target = stowed - (stowed - full) * frac;
        let mpc = AirBrakesMPC::new(rocket.clone(), target);
        let mut s = State {
            altitude_asl: start.altitude_asl,
            velocity: start.velocity,
        };
        let dt = 0.02f32;
        let mut ticks = 0usize;
        let mut ext = 0.0f32;
        while s.velocity.y > 0.0 {
            if ticks % 5 == 0 {
                ext = mpc.update(s.altitude_asl, s.velocity).extension_percentage;
            }
            // extension 0..1 maps onto the dynamics' drag percentage -1..1
            let Derivative(k) = calculate_state_derivatives(ext * 2.0 - 1.0, &s, &rocket);
            s = State {
                altitude_asl: s.altitude_asl + k.altitude_asl * dt,
                velocity: s.velocity + k.velocity * dt,
            };
            ticks += 1;
        }
        let err = s.altitude_asl - target;
        eprintln!(
            "authority: target {target:.0} m ASL ({:.0}% of authority) -> reached \
             {:.0} m, error {err:+.0} m",
            frac * 100.0,
            s.altitude_asl
        );
        assert!(
            err.abs() < 100.0,
            "closed loop missed a reachable target by {err:+.0} m"
        );
    }
}

/// Stage 1 — the thrust-vector alignment — is the one place where a clipped
/// accelerometer would do permanent damage. It averages the measured
/// specific force over the first 0.5 s of boost and takes the mean
/// DIRECTION as the airframe axis, producing two values that are latched
/// once and never revisited: `q_av_to_rocket` (how the board is mounted)
/// and `thrust_axis_av` (the axis the burnout latch projects onto). Nothing
/// downstream re-derives them, so an error there is an error for the whole
/// flight — unlike the dead-reckoned velocity, which the barometer erases
/// within a second of birth.
///
/// What saves it on Osiris is timing, and only just. The axial specific
/// force during the alignment window is 14.8-15.5 g, and the +-16 g rail is
/// not reached until ignition+0.86 s — a third of a second after Stage 1
/// has already closed. This test pins that ordering, because it is the
/// margin the whole "clipping is survivable" argument rests on.
#[test]
fn stage1_alignment_finishes_before_the_accelerometer_rails() {
    init_logger();
    const STAGE1_DURATION_S: f32 = 0.5;
    let truth = Truth::load(O3400_CSV);

    // Where the airframe actually crosses the rail, from OpenRocket's own
    // forces rather than from the sensor model.
    let rail = 16.0 * 9.81;
    let first_over = truth
        .rows
        .iter()
        .find(|r| (r.thrust - r.drag) / r.mass > rail)
        .map(|r| r.t)
        .expect("this motor never reaches 16 g — the premise has changed");
    let peak_in_window = truth
        .rows
        .iter()
        .filter(|r| r.t <= STAGE1_DURATION_S)
        .map(|r| (r.thrust - r.drag) / r.mass)
        .fold(0.0f32, f32::max);
    eprintln!(
        "stage 1: alignment window is 0..{STAGE1_DURATION_S}s, peaking at {:.2} g; \
         the airframe first passes 16 g at {first_over:.3}s",
        peak_in_window / 9.81
    );
    assert!(
        first_over > STAGE1_DURATION_S,
        "the airframe reaches the +-16 g rail at {first_over}s, INSIDE the {STAGE1_DURATION_S}s \
         thrust-vector alignment window — the mean thrust direction would be \
         taken from clipped samples and the mounting calibration would be \
         wrong for the entire flight"
    );

    // And the same thing through the actual sensor model, worst-case mount.
    let samples = synthesize(
        &truth,
        &SensorModel {
            until_s: truth.apogee().0 + 5.0,
            accel_full_scale: rail,
            mount: axis_aligned_mounting(),
            ..Default::default()
        },
    );
    let r = replay(&samples, osiris_config(), 0.0);
    let first_clip = r.first_clip_t.expect("nothing clipped");
    eprintln!("stage 1: first clipped sample at ignition+{first_clip:.3}s");
    assert!(
        first_clip > STAGE1_DURATION_S,
        "a sample clipped at {first_clip}s, inside the alignment window"
    );

    // The observable consequence: tilt. A misaligned axis shows up as a
    // constant tilt bias for the whole flight, and tilt is what sets the
    // horizontal component of the velocity handed to the MPC.
    let (worst, worst_t) = r.tilt_track.iter().fold((0.0f32, 0.0f32), |acc, (t, e, tr)| {
        let d = (e - tr).abs();
        if d > acc.0 { (d, *t) } else { acc }
    });
    eprintln!(
        "stage 1: worst tilt error {:.2} deg (at t={worst_t:.1}s) over {} samples, \
         with the accelerometer railing for {:.2}s of the burn",
        worst.to_degrees(),
        r.tilt_track.len(),
        r.clipped_samples as f32 / 416.0
    );
    assert!(
        worst.to_degrees() < 5.0,
        "tilt drifted {:.1} deg — the alignment or the gyro integration is off",
        worst.to_degrees()
    );
}

/// The avionics can be bolted into the airframe any way up, and nothing in
/// the firmware is told which way. The estimator is supposed to work that
/// out for itself: pad gravity gives it "down", the Stage-1 thrust average
/// gives it the airframe axis, and those two together define the mounting.
/// There is no per-board axis configuration anywhere, which is a strong
/// claim — this is the test of it.
///
/// Six orientations, from bolted-flat to fully inverted to a deliberately
/// awkward compound angle. Every one has to reach the same flight: pad
/// calibration completes, the lockout opens on the drag check rather than
/// the timeout, it opens only after the airframe is genuinely subsonic, and
/// apogee lands on truth.
///
/// Upside-down matters more than it looks. Inverted, pad gravity reads
/// -9.8 on the mounting axis while thrust reads +145 on the same axis, so
/// any code that assumed the two share a sign — or that took a magnitude
/// where it needed a direction — inverts the whole attitude solution.
#[test]
fn any_mounting_orientation_flies_the_same_flight() {
    init_logger();
    let truth = Truth::load(O3400_CSV);
    let (apogee_t, apogee_asl) = truth.apogee();
    let m08 = truth.mach_down_crossing(0.8);

    let orientations: [(&str, UnitQuaternion<f32>); 6] = [
        ("flat, axis on +Z", UnitQuaternion::identity()),
        (
            "rolled 90 deg about the axis",
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), core::f32::consts::FRAC_PI_2),
        ),
        (
            "on its side, axis on +Y",
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f32::consts::FRAC_PI_2),
        ),
        (
            "on its side, axis on -X",
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), core::f32::consts::FRAC_PI_2),
        ),
        (
            "inverted, axis on -Z",
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f32::consts::PI),
        ),
        ("awkward compound angle", imu_mounting()),
    ];

    for (name, mount) in orientations {
        let samples = synthesize(
            &truth,
            &SensorModel {
                until_s: apogee_t + 5.0,
                mount,
                ..Default::default()
            },
        );
        let r = replay(&samples, osiris_config(), apogee_asl - 150.0);

        let cal = r
            .calibration_t
            .unwrap_or_else(|| panic!("{name}: pad calibration never completed"));
        assert!(cal < 0.0, "{name}: calibration only completed at {cal}s");

        let (born, forced) = r
            .birth
            .unwrap_or_else(|| panic!("{name}: filter never born"));
        let (ab_t, ab_alt) = r
            .estimated_apogee()
            .unwrap_or_else(|| panic!("{name}: no altitude reported"));
        let worst_tilt = r
            .tilt_track
            .iter()
            .filter(|(t, _, _)| *t < apogee_t - 2.0)
            .map(|(_, e, tr)| (e - tr).abs())
            .fold(0.0f32, f32::max);
        let (vv_err, n) = mean_error(&r.vv_track, born + 2.0, apogee_t - 2.0);

        eprintln!(
            "{name:32} born {born:5.2}s (forced {forced:5}) | tilt err <= {:.2} deg | \
             apogee {:+.2}s / {:+4.0}m | vv err {vv_err:.2} m/s",
            worst_tilt.to_degrees(),
            ab_t - apogee_t,
            ab_alt - apogee_asl,
        );

        assert!(!forced, "{name}: birth fell through to the T_max timeout");
        assert!(
            born > m08,
            "{name}: born at {born}s while still above Mach 0.8 ({m08}s)"
        );
        for (start, end) in &r.subsonic_spans {
            assert!(
                *start >= m08,
                "{name}: drag check read subsonic at {start}s..{end}s, still supersonic"
            );
        }
        assert!(
            worst_tilt.to_degrees() < 5.0,
            "{name}: tilt drifted {:.1} deg — the mounting was not solved",
            worst_tilt.to_degrees()
        );
        assert!(
            (ab_t - apogee_t).abs() < 3.0 && (ab_alt - apogee_asl).abs() < 80.0,
            "{name}: apogee {:+.2}s / {:+.0}m off truth",
            ab_t - apogee_t,
            ab_alt - apogee_asl
        );
        assert!(n > 2000 && vv_err < 8.0, "{name}: coast vv error {vv_err}");
    }
}

/// `approximate_air_density` against an exact f64 ISA reference.
///
/// This test only means anything because the implementation is now
/// target-independent. It used to be written `x.powf(y)`, which resolves to
/// the inherent `f32::powf` under std and to whatever `F32Ext` trait is in
/// scope under `no_std` — so this suite validated arithmetic the rocket
/// never ran, and the rocket's air density was up to 39% low at altitude.
/// That inflated the Mach-lockout drag inversion by 28% and pushed the
/// lockout exit 3 s late on the bench, and it fed the MPC's apogee
/// prediction, where under-reading density over-predicts apogee and
/// over-extends the brakes.
///
/// `libm::powf` called by name has no inherent-vs-trait resolution, so what
/// this checks is what flies.
#[test]
fn air_density_matches_the_isa_reference() {
    init_logger();
    use crate::utils::approximate_air_density;

    // ISA troposphere in f64, written independently of the implementation.
    let reference = |alt: f64| 1.225 * (1.0 - 2.25577e-5 * alt).powf(4.256);

    let mut worst = 0.0f64;
    let mut worst_alt = 0.0f64;
    eprintln!("  alt |   reference | implementation |    error");
    for step in 0..=24 {
        let alt = step as f64 * 500.0;
        let got = approximate_air_density(alt as f32) as f64;
        let r = reference(alt);
        let err = (got - r) / r;
        if err.abs() > worst.abs() {
            worst = err;
            worst_alt = alt;
        }
        if step % 4 == 0 {
            eprintln!("{alt:5.0} | {r:11.6} | {got:14.6} | {:+.5}%", err * 100.0);
        }
    }
    eprintln!("\nworst {:+.5}% at {worst_alt:.0} m over 0-12 km", worst * 100.0);
    // libm's powf is sub-ulp; what is left is f32 rounding of the inputs.
    assert!(
        worst.abs() < 1e-4,
        "density is {:+.4}% off ISA at {worst_alt:.0} m — check that nothing \
         reintroduced a method-call `powf`, which changes meaning between the \
         host and the board",
        worst * 100.0
    );
}

// ---------------------------------------------------------------------------
// Diagnostic, not a regression test (run with --ignored --nocapture): what
// does the ignition threshold cost on THIS airframe's motors?
//
// The same sweep over the two archived flight logs was retired on
// 2026-08-17; its answer is transcribed into
// `AirbrakesConfig::ignition_detection_acc_threshold` (on LC'25's softer
// curve 8 g costs 0.45 s and 10 g never latches at all). This one answers it
// on the two simulated Osiris motors, which is where `FLIGHT_CONFIG`'s 8 g
// actually has to hold. Since 2026-08-17 both halves run the same detector
// (`crate::ignition_detector`), so one latch time describes both — the only
// thing that could separate them is a different threshold, which is exactly
// what this sweeps.
//
// Times are seconds after TRUE ignition, so they are the detector's lag,
// which is also the error in the origin of every Mach lockout timer.
// ---------------------------------------------------------------------------
#[test]
#[ignore]
fn ignition_latch_time_by_threshold() {
    init_logger();

    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let samples = synthesize(&truth, &SensorModel::default());
        eprintln!("--- {path} ---");

        let mut reference: Option<f32> = None;
        for g in [4.0f32, 6.0, 8.0, 10.0, 12.0] {
            let mut cfg = osiris_config();
            cfg.profile.ignition_detection_acc_threshold = g * 9.81;
            cfg.airbrakes.ignition_detection_acc_threshold = g * 9.81;
            let mut est = FlightEstimators::new(cfg);

            let mut pyro_t: Option<f32> = None;
            let mut ab_t: Option<f32> = None;
            for s in &samples {
                let _ = est.update(s.t_us, Some(&s.imu), s.baro_altitude_asl);
                if pyro_t.is_none() && !matches!(est.state(), crate::RocketState::OnPad) {
                    pyro_t = Some(s.truth_t);
                }
                if ab_t.is_none()
                    && est
                        .airbrakes_estimator()
                        .is_some_and(|ab| ab.ignition_latched())
                {
                    ab_t = Some(s.truth_t);
                }
                if pyro_t.is_some() && ab_t.is_some() {
                    break;
                }
            }

            if g == 4.0 {
                reference = pyro_t;
            }
            let cost = match (pyro_t, reference) {
                (Some(t), Some(r)) => format!("{:+.3} s vs 4 g", t - r),
                (None, _) => "NEVER LATCHED".into(),
                _ => "-".into(),
            };
            eprintln!(
                "  {g:>4.1} g : pyro half {pyro_t:?}, airbrakes half {ab_t:?}  ({cost})"
            );
            assert_eq!(
                pyro_t, ab_t,
                "the two halves ran the same detector at the same threshold and \
                 still disagreed about when the motor lit"
            );
        }
    }
}

/// TEMPORARY bandwidth study (delete when it has answered its question).
///
/// The archived-flight study scores against a smoothed version of the same
/// barometer, which structurally flatters any filter that follows the baro
/// harder. Here the truth is OpenRocket's own trajectory, so that confound
/// is gone: `vv_track`/`alt_track` already carry (estimate, truth) pairs,
/// and the same synthesized baro stream feeds a sweep of baro-ONLY filters
/// for comparison.
#[test]
fn kf_bandwidth_vs_truth() {
    init_logger();
    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let (apogee_t, apogee_asl) = truth.apogee();
        let samples = synthesize(
            &truth,
            &SensorModel {
                until_s: apogee_t + 5.0,
                ..Default::default()
            },
        );
        let r = replay(&samples, osiris_config(), apogee_asl - 150.0);

        // score from birth+0.5 s to 2 s before apogee
        let (birth_t, _) = r.birth.expect("no birth");
        let from = birth_t + 0.5;
        let to = apogee_t - 2.0;
        let score = |track: &[(f32, f32, f32)]| -> (f32, f32, usize) {
            let (mut sum, mut max, mut n) = (0.0f32, 0.0f32, 0usize);
            for (t, est, tru) in track {
                if *t >= from && *t <= to {
                    sum += (est - tru).abs();
                    max = max.max((est - tru).abs());
                    n += 1;
                }
            }
            (sum / n.max(1) as f32, max, n)
        };
        let (vv_mean, vv_max, n) = score(&r.vv_track);
        let (alt_mean, alt_max, _) = score(&r.alt_track);
        let jitter = {
            let (mut ss, mut cnt) = (0.0f32, 0usize);
            for w in r.vv_track.windows(2) {
                if w[0].0 >= from && w[1].0 <= to && w[1].0 > w[0].0 {
                    let d = (w[1].1 - w[0].1) / (w[1].0 - w[0].0) * 0.01;
                    ss += d * d;
                    cnt += 1;
                }
            }
            (ss / cnt.max(1) as f32).sqrt()
        };
        eprintln!("\n=== {path} (truth-scored, {n} samples, {from:.1}..{to:.1}s) ===");
        eprintln!(
            "  {:<24} {:>8} {:>8} {:>8} {:>8} {:>9}",
            "estimator", "|dvv|", "max", "|dalt|", "max", "jitter"
        );
        eprintln!(
            "  {:<24} {vv_mean:>8.2} {vv_max:>8.2} {alt_mean:>8.2} {alt_max:>8.2} {jitter:>9.3}",
            "flown (IMU-aided)"
        );

        // baro-only sweep on the SAME synthesized baro stream, born at the
        // same instant with a causal slope seed
        for tau in [0.10f32, 0.20, 0.35, 0.50, 0.75, 1.00, 1.73] {
            let born_i = samples
                .iter()
                .position(|s| s.truth_t >= birth_t)
                .expect("birth outside samples");
            // causal 0.5 s slope seed
            let (mut st, mut sy, mut stt, mut sty, mut cnt) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0f64);
            let t0 = samples[born_i].truth_t;
            for s in samples[..=born_i].iter().rev() {
                if t0 - s.truth_t > 0.5 {
                    break;
                }
                let t = (s.truth_t - t0) as f64;
                let y = s.baro_altitude_asl as f64;
                st += t;
                sy += y;
                stt += t * t;
                sty += t * y;
                cnt += 1.0;
            }
            let den = cnt * stt - st * st;
            let slope = if den.abs() > 1e-9 { (cnt * sty - st * sy) / den } else { 0.0 };
            let intercept = if cnt > 0.0 { (sy - slope * st) / cnt } else { 0.0 };

            let (mut alt, mut vel) = (intercept as f32, slope as f32);
            let (mut p00, mut p01, mut p10, mut p11) = (9.0f32, 0.0, 0.0, 900.0);
            let (r_std, q_std) = (3.0f32, 3.0f32 / (2.0 * tau * tau));
            let (mut sum, mut mx, mut nn) = (0.0f32, 0.0f32, 0usize);
            let (mut asum, mut amx) = (0.0f32, 0.0f32);
            let (mut ss, mut jn, mut prev) = (0.0f32, 0usize, None::<(f32, f32)>);
            for w in samples[born_i..].windows(2) {
                let (prev_s, s) = (&w[0], &w[1]);
                let dt = (s.truth_t - prev_s.truth_t).clamp(0.0, 0.25);
                alt += vel * dt;
                let q = q_std * q_std;
                let (a00, a01, a10, a11) = (p00, p01, p10, p11);
                p00 = a00 + dt * (a01 + a10) + dt * dt * a11 + q * dt.powi(4) / 4.0;
                p01 = a01 + dt * a11 + q * dt.powi(3) / 2.0;
                p10 = a10 + dt * a11 + q * dt.powi(3) / 2.0;
                p11 = a11 + q * dt * dt;
                let innovation = s.baro_altitude_asl - alt;
                if innovation.abs() <= 100.0 {
                    let rr = r_std * r_std;
                    let sden = p00 + rr;
                    let (k0, k1) = (p00 / sden, p10 / sden);
                    alt += k0 * innovation;
                    vel += k1 * innovation;
                    let (b00, b01, b10, b11) = (p00, p01, p10, p11);
                    let a = 1.0 - k0;
                    p00 = a * a * b00 + k0 * k0 * rr;
                    p01 = a * (b01 - k1 * b00) + k0 * k1 * rr;
                    p10 = a * (b10 - k1 * b00) + k0 * k1 * rr;
                    p11 = b11 - k1 * (b01 + b10) + k1 * k1 * b00 + k1 * k1 * rr;
                }
                if s.truth_t >= from && s.truth_t <= to {
                    sum += (vel - s.truth.vv).abs();
                    mx = mx.max((vel - s.truth.vv).abs());
                    asum += (alt - s.truth.altitude_asl).abs();
                    amx = amx.max((alt - s.truth.altitude_asl).abs());
                    nn += 1;
                    if let Some((pt, pv)) = prev
                        && s.truth_t > pt
                    {
                        let d = (vel - pv) / (s.truth_t - pt) * 0.01;
                        ss += d * d;
                        jn += 1;
                    }
                    prev = Some((s.truth_t, vel));
                }
            }
            eprintln!(
                "  {:<24} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>9.3}",
                format!("baro-only tau={tau:.2}s"),
                sum / nn.max(1) as f32,
                mx,
                asum / nn.max(1) as f32,
                amx,
                (ss / jn.max(1) as f32).sqrt(),
            );
        }
    }
}

/// TEMPORARY: does the MPC's own input tolerate a baro-only source?
///
/// Scored at the MPC's level rather than the state's: commanded flap
/// extension and predicted apogee against an oracle MPC fed OpenRocket's
/// true state, over exactly the window the real gate hands states out.
/// Rows 1 and 2 swap ONE channel each, so altitude and velocity can be
/// blamed separately.
#[test]
fn mpc_input_baro_only() {
    init_logger();

    /// Plain 2-state constant-velocity KF on baro alone -- no accel input.
    struct BaroOnly {
        alt: f32,
        vel: f32,
        p: [f32; 4],
        q: f32,
        r: f32,
    }
    impl BaroOnly {
        fn step(&mut self, z: f32, dt: f32) {
            self.alt += self.vel * dt;
            let (p00, p01, p10, p11) = (self.p[0], self.p[1], self.p[2], self.p[3]);
            let q = self.q * self.q;
            self.p[0] = p00 + dt * (p01 + p10) + dt * dt * p11 + q * dt.powi(4) / 4.0;
            self.p[1] = p01 + dt * p11 + q * dt.powi(3) / 2.0;
            self.p[2] = p10 + dt * p11 + q * dt.powi(3) / 2.0;
            self.p[3] = p11 + q * dt * dt;
            let innovation = z - self.alt;
            if innovation.abs() > 100.0 {
                return;
            }
            let rr = self.r * self.r;
            let s = self.p[0] + rr;
            let (k0, k1) = (self.p[0] / s, self.p[2] / s);
            self.alt += k0 * innovation;
            self.vel += k1 * innovation;
            let (b00, b01, b10, b11) = (self.p[0], self.p[1], self.p[2], self.p[3]);
            let a = 1.0 - k0;
            self.p[0] = a * a * b00 + k0 * k0 * rr;
            self.p[1] = a * (b01 - k1 * b00) + k0 * k1 * rr;
            self.p[2] = a * (b10 - k1 * b00) + k0 * k1 * rr;
            self.p[3] = b11 - k1 * (b01 + b10) + k1 * k1 * b00 + k1 * k1 * rr;
        }
    }

    const TAUS: [f32; 4] = [0.10, 0.20, 0.35, 0.50];

    for path in [O3400_CSV, N2900_CSV] {
        let truth = Truth::load(path);
        let (apogee_t, apogee_asl) = truth.apogee();
        let samples = synthesize(
            &truth,
            &SensorModel {
                until_s: apogee_t + 5.0,
                ..Default::default()
            },
        );
        let cfg = osiris_config();
        let target_asl = apogee_asl - 150.0;
        let mpc = AirBrakesMPC::new(cfg.airbrakes.rocket.clone(), target_asl);
        let mut est = FlightEstimators::new(cfg);

        let mut filters: [Option<BaroOnly>; 4] = [None, None, None, None];
        let mut baro_hist: std::collections::VecDeque<(f32, f32)> = Default::default();
        let mut ticks: Vec<[Option<(f32, f32)>; 6]> = Vec::new();
        let mut prev_t: Option<f32> = None;
        let mut gate_open_t: Option<f32> = None;
        let mut gate_last_t = 0.0f32;

        for s in &samples {
            let _ = est.update(s.t_us, Some(&s.imu), s.baro_altitude_asl);
            let dt = prev_t.map(|p| (s.truth_t - p).clamp(0.0, 0.25)).unwrap_or(0.0);
            prev_t = Some(s.truth_t);

            baro_hist.push_back((s.truth_t, s.baro_altitude_asl));
            while baro_hist.front().is_some_and(|(t, _)| s.truth_t - t > 0.6) {
                baro_hist.pop_front();
            }

            let gated = est.airbrakes_mpc_states();
            for (k, f) in filters.iter_mut().enumerate() {
                match f {
                    Some(f) => f.step(s.baro_altitude_asl, dt),
                    None if gated.is_some() => {
                        // Born when the real gate opens, seeded the way a
                        // baro-only design actually would: causal
                        // least-squares slope of its own last 0.5 s.
                        let (mut st, mut sy, mut stt, mut sty, mut cnt) =
                            (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
                        for (bt, by) in baro_hist.iter() {
                            let t = (bt - s.truth_t) as f64;
                            st += t;
                            sy += *by as f64;
                            stt += t * t;
                            sty += t * *by as f64;
                            cnt += 1.0;
                        }
                        let den = cnt * stt - st * st;
                        let slope =
                            if den.abs() > 1e-9 { (cnt * sty - st * sy) / den } else { 0.0 };
                        *f = Some(BaroOnly {
                            alt: s.baro_altitude_asl,
                            vel: slope as f32,
                            p: [9.0, 0.0, 0.0, 30.0 * 30.0],
                            q: 3.0 / (2.0 * TAUS[k] * TAUS[k]),
                            r: 3.0,
                        });
                    }
                    None => {}
                }
            }

            let Some(states) = gated else { continue };
            gate_open_t.get_or_insert(s.truth_t);
            gate_last_t = s.truth_t;
            // let every candidate settle for 1 s -- the birth transient is
            // not what this is measuring
            if s.truth_t - gate_open_t.unwrap() < 1.0 {
                continue;
            }

            let oracle = mpc.update(
                s.truth.altitude_asl,
                Vector2::new(s.truth.lateral_velocity, s.truth.vv),
            );
            let mut tick: [Option<(f32, f32)>; 6] = [None; 6];
            let mut put = |slot: usize, sol: crate::MpcSolution| {
                tick[slot] = Some((
                    (sol.extension_percentage - oracle.extension_percentage).abs(),
                    (sol.predicted_apogee_asl - oracle.predicted_apogee_asl).abs(),
                ));
            };

            put(0, mpc.update(states.altitude_asl, states.velocity));
            if let Some(f) = &filters[1] {
                // one channel swapped at a time, both at tau = 0.20 s
                put(1, mpc.update(f.alt, states.velocity));
                put(2, mpc.update(states.altitude_asl, Vector2::new(0.0, f.vel)));
            }
            for (slot, k) in [(3usize, 0usize), (4, 1), (5, 3)] {
                if let Some(f) = &filters[k] {
                    put(slot, mpc.update(f.alt, Vector2::new(0.0, f.vel)));
                }
            }
            ticks.push(tick);
        }

        eprintln!(
            "\n=== {path}: MPC input study, gate open {:.1}..{gate_last_t:.1}s, {} scored ticks, \
             target {target_asl:.0} m ASL ===",
            gate_open_t.unwrap(),
            ticks.len(),
        );
        eprintln!(
            "  {:<38} {:>8} {:>8} {:>10} {:>9}",
            "MPC fed from", "|dext|", "p95", "|dapogee|", "p95"
        );
        eprintln!("  {:<38} {:>8} {:>8} {:>10} {:>9}", "", "%pt", "%pt", "m", "m");
        for (slot, label) in [
            (0usize, "flown (IMU+baro KF)  <- ships today"),
            (1, "baro-only ALTITUDE + flown velocity"),
            (2, "flown altitude + baro-only VELOCITY"),
            (3, "baro-only both, tau=0.10s"),
            (4, "baro-only both, tau=0.20s"),
            (5, "baro-only both, tau=0.50s"),
        ] {
            let (mut ext, mut apo): (Vec<f32>, Vec<f32>) = (Vec::new(), Vec::new());
            for t in &ticks {
                if let Some((e, a)) = t[slot] {
                    ext.push(e);
                    apo.push(a);
                }
            }
            if ext.is_empty() {
                continue;
            }
            ext.sort_by(f32::total_cmp);
            apo.sort_by(f32::total_cmp);
            let p95 = |v: &[f32]| v[((v.len() as f32 * 0.95) as usize).min(v.len() - 1)];
            let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len() as f32;
            eprintln!(
                "  {:<38} {:>8.2} {:>8.2} {:>10.1} {:>9.1}",
                label,
                mean(&ext) * 100.0,
                p95(&ext) * 100.0,
                mean(&apo),
                p95(&apo),
            );
        }
    }
}


