use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use super::DT;

/// A simple dead-reckoning filter that tracks orientation, position, and velocity
/// in an inertial frame using IMU measurements in the device frame.
#[derive(Debug, Clone)]
pub struct DeadReckoner {
    /// Rotation from inertial frame to device frame
    pub orientation: UnitQuaternion<f32>,
    /// Position in inertial frame (meters)
    pub position: Vector3<f32>,
    /// Velocity in inertial frame (m/s)
    pub velocity: Vector3<f32>,

    gravity: f32,
}

impl DeadReckoner {
    /// Initialize with a given orientation. Position and velocity start at zero.
    ///
    /// # Arguments
    /// * `initial_orientation` - quaternion rotating inertial → device
    pub fn new(initial_orientation: UnitQuaternion<f32>) -> Self {
        Self {
            orientation: initial_orientation,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            gravity: 9.81,
        }
    }

    pub fn new_no_gravity(initial_orientation: UnitQuaternion<f32>) -> Self {
        Self {
            orientation: initial_orientation,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            gravity: 0.0,
        }
    }

    /// Update the dead reckoning state with one IMU sample.
    ///
    /// # Arguments
    /// * `accel` - accelerometer measurement (specific force) in device frame (m/s²)
    /// * `gyro`  - angular rate measurement in device frame (rad/s)
    pub fn update(&mut self, accel: &Vector3<f32>, gyro: &Vector3<f32>) {
        // 1) Integrate orientation: quaternion exponential via small-angle approx
        let delta_orientation =
            UnitQuaternion::from_quaternion(Quaternion::from_parts(1.0, -gyro * DT / 2.0));
        self.orientation = delta_orientation * self.orientation;

        // 2) Rotate accel into inertial frame and add gravity to get linear accel
        let mut accel_inertial = self.orientation.inverse_transform_vector(accel);
        accel_inertial.z -= self.gravity;

        // 3) Integrate velocity and position
        self.position += self.velocity * DT + accel_inertial * (0.5 * DT * DT);
        self.velocity += accel_inertial * DT;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::init_logger;

    #[test]
    fn test() {
        init_logger();

        let mut reckoner = DeadReckoner::new_no_gravity(UnitQuaternion::identity());

        // rotate by x axis 90 degrees
        let rotation_speed_deg = 10f32;
        let duration = 90f32 / rotation_speed_deg;
        let ticks = (duration / DT).round() as usize;
        let gyro_measurement = Vector3::new(rotation_speed_deg.to_radians(), 0.0, 0.0);
        let acc_measurement = Vector3::new(0.0, 0.0, 0.0);

        for _ in 0..ticks {
            reckoner.update(&acc_measurement, &gyro_measurement);
        }

        // should be "UnitQuaternion angle: 1.5707971 − axis: (-1, 0, 0)"
        log_info!("orientation: {}", reckoner.orientation);

        // move to z axis
        reckoner.update(&Vector3::new(0.0, 0.0, 10000.0), &Vector3::zeros());
        log_info!("position: {}", reckoner.position);
        assert!(reckoner.position.y < -0.01);
    }
}
