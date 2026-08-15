use nalgebra::{UnitQuaternion, Vector3};

/// Dead reckoning: track orientation, velocity, and position by adding up
/// IMU readings step by step, with no outside correction. Orientation is
/// gyro-only (the accelerometer never touches it); velocity/position
/// integrate the accelerometer rotated into the earth frame.
///
/// Every update takes the measured time step — nothing here assumes a
/// fixed sample rate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct DeadReckoner {
    /// Rotation from earth (inertial) frame to device frame
    pub orientation: UnitQuaternion<f32>,
    /// Position in earth frame (meters); z is altitude ASL
    pub position: Vector3<f32>,
    /// Velocity in earth frame (m/s)
    pub velocity: Vector3<f32>,
    /// Latest linear acceleration in earth frame (gravity removed, m/s^2)
    pub acceleration: Vector3<f32>,
}

impl DeadReckoner {
    /// Initialize with a given orientation (earth -> device). Position and
    /// velocity start at zero; set `position.z` to the pad altitude after
    /// construction.
    pub fn new(initial_orientation: UnitQuaternion<f32>) -> Self {
        Self {
            orientation: initial_orientation,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        }
    }

    /// Integrate one IMU sample over the measured time step `dt` (s).
    ///
    /// * `accel` - specific force in device frame (m/s^2)
    /// * `gyro`  - angular rate in device frame (rad/s), bias already removed
    pub fn update(&mut self, accel: &Vector3<f32>, gyro: &Vector3<f32>, dt: f32) {
        // 1) Orientation: quaternion exponential via small-angle approx
        let delta_orientation = UnitQuaternion::from_scaled_axis(gyro * dt);
        self.orientation = self.orientation * delta_orientation;

        // 2) Rotate accel into the earth frame, remove gravity
        let mut accel_earth = self.orientation.transform_vector(accel);
        accel_earth.z -= 9.81;
        self.acceleration = accel_earth;

        // 3) Velocity and position
        self.position += self.velocity * dt + accel_earth * (0.5 * dt * dt);
        self.velocity += accel_earth * dt;
    }
}
