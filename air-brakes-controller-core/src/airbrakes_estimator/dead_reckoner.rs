use nalgebra::Vector3;

/// Dead reckoning: track which way is up, and the vertical channel of
/// acceleration and velocity, by adding up IMU readings step by step with
/// no outside correction. Attitude is gyro-only (the accelerometer never
/// touches it after the pad).
///
/// It does NOT track altitude, since 2026-08-17. Nothing ever read the
/// integrated altitude where it mattered: the vertical filter is born from a
/// median of the buffered BARO, never from here, so the one number this
/// reckoner hands across the birth boundary is the velocity. Its two other
/// readers are gone — the lockout-exit drag check now evaluates the
/// atmosphere at a configured constant
/// ([`MachLockoutConfig::subsonic_crossing_altitude_asl`]), and the
/// pre-birth altitude is no longer published to the log or the downlink.
/// Deleting it removes a doubly-integrated quantity that looked like a
/// position fix and was never used as one.
///
/// Only the vertical is tracked, because only the vertical is ever read.
/// A full attitude was carried as a quaternion, with 3-axis position and
/// velocity beside it, until 2026-08-17; the horizontal components were
/// dead the whole time, and the earth frame's azimuth they were expressed
/// in was arbitrary anyway — the pad attitude is solved from gravity alone,
/// and gravity says nothing about heading.
///
/// One vector is enough because both questions the estimator asks of an
/// attitude are about the vertical:
///
/// * the earth-frame vertical specific force is the third ROW of the
///   device->earth rotation dotted with the reading — and that row is
///   exactly [`Self::up_av`];
/// * the airframe's tilt is the angle between [`Self::up_av`] and the
///   thrust axis, which is a constant in the device frame.
///
/// It also removes a latent all-NaN attitude. The pad orientation used to
/// come from a minimal-arc rotation about `gravity x UP`, which is the zero
/// vector — and normalizes to NaN — for an exactly inverted mounting.
/// `up_av` is just the normalized pad gravity, so upside-down is an
/// ordinary case with no special branch and no near-degenerate axis.
///
/// Every update takes the measured time step — nothing here assumes a
/// fixed sample rate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct DeadReckoner {
    /// Earth UP written in the device (avionics) frame, unit length —
    /// equivalently, the direction the accelerometer reads +1 g along while
    /// the airframe is still.
    pub up_av: Vector3<f32>,
    /// Vertical velocity in the earth frame (m/s), + is up.
    pub vertical_velocity: f32,
    /// Latest vertical linear acceleration in the earth frame (gravity
    /// removed, m/s^2).
    pub vertical_acceleration: f32,
}

impl DeadReckoner {
    /// Start from the pad: `up_av` is earth UP in the device frame (the
    /// normalized pad gravity — see [`Self::up_av`]). Vertical velocity
    /// starts at zero, which is the whole of the initial condition — there
    /// is no altitude to anchor.
    pub fn new(up_av: Vector3<f32>) -> Self {
        Self {
            up_av,
            vertical_velocity: 0.0,
            vertical_acceleration: 0.0,
        }
    }

    /// Integrate one IMU sample over the measured time step `dt` (s).
    ///
    /// * `accel` - specific force in device frame (m/s^2)
    /// * `gyro`  - angular rate in device frame (rad/s), bias already removed
    pub fn update(&mut self, accel: &Vector3<f32>, gyro: &Vector3<f32>, dt: f32) {
        // 1) Attitude: the device frame turns by `gyro * dt`, so a vector
        //    held fixed in the EARTH frame turns the other way when written
        //    in device coordinates.
        self.up_av = rotate(&self.up_av, &(gyro * -dt));

        // 2) Vertical specific force, gravity removed. Rotating `accel`
        //    into the earth frame and keeping z is the same arithmetic as
        //    this one dot product.
        let vertical_accel = self.up_av.dot(accel) - 9.81;
        self.vertical_acceleration = vertical_accel;

        // 3) Velocity
        self.vertical_velocity += vertical_accel * dt;
    }
}

/// Rotate `v` by the rotation vector `w` (axis * angle, radians), by
/// Rodrigues written in HALF angles:
/// `v + 2 sin(h) [ cos(h) (k x v) + sin(h) (k (k.v) - v) ]`, `h = |w|/2`.
///
/// The half angles are not cosmetic. Written the textbook way —
/// `v cos(t) + sin(t) (k x v) + (1 - cos(t)) k (k.v)` — `cos(t)` rounds to
/// exactly 1.0 in f32 at the step sizes this runs at (a 3 deg/s rail at
/// 416 Hz is t = 1.3e-4 rad), so the term that shrinks `v` back to unit
/// length vanishes into the rounding and the vector grows without bound.
/// Measured over 10^6 steps of a fast tumble: worst `| |v| - 1 |` is
/// 8.4e-3 written that way against 1.7e-5 written this way, and the
/// attitude error 0.61 deg against 0.026 deg.
///
/// Against the composed unit quaternion this replaces, the same measurement
/// gives 1.7e-2 (quaternion) against 1.7e-5 — nalgebra composes with
/// `Unit::new_unchecked` and never renormalizes, so its norm drifts and the
/// transform then amplifies that drift. Which is why nothing here
/// renormalizes either: over a whole flight (~8300 steps) the worst norm
/// error measured is 2.7e-6, and a margin that wide does not need a
/// mechanism guarding it.
fn rotate(v: &Vector3<f32>, w: &Vector3<f32>) -> Vector3<f32> {
    let angle = w.magnitude();
    if angle == 0.0 {
        // Perfectly zero rate (or dt): `w / angle` would be NaN.
        return *v;
    }
    let k = w / angle;
    let (sin_h, cos_h) = (libm::sinf(0.5 * angle), libm::cosf(0.5 * angle));
    v + 2.0 * sin_h * (cos_h * k.cross(v) + sin_h * (k * k.dot(v) - v))
}
