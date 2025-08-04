use nalgebra::{UnitQuaternion, Vector3};

use crate::state_estimator::ascent_fusion::DT;

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
        let delta_orientation = UnitQuaternion::from_scaled_axis(gyro * DT);
        self.orientation = self.orientation * delta_orientation;

        // 2) Rotate accel into inertial frame and add gravity to get linear accel
        let mut accel_inertial = self.orientation.transform_vector(accel);
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
    fn test_dead_reckoner() {
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

        // should be "UnitQuaternion angle: 1.5707971 − axis: (1, 0, 0)"
        log_info!("orientation: {}", reckoner.orientation);

        // move to z axis
        reckoner.update(&Vector3::new(0.0, 0.0, 10000.0), &Vector3::zeros());
        log_info!("position: {}", reckoner.position);
        assert!(reckoner.position.y < -0.01);
    }

    #[test]
    fn test_dead_reckoner2() {
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
        log_info!(
            "orientation: {} {:?}",
            reckoner.orientation,
            reckoner.orientation
        );

        // rotate by y axis (world's z axis) 90 degrees
        let gyro_measurement = Vector3::new(0.0, rotation_speed_deg.to_radians(), 0.0);

        for _ in 0..ticks {
            reckoner.update(&acc_measurement, &gyro_measurement);
        }

        log_info!(
            "orientation: {} {:?}",
            reckoner.orientation,
            reckoner.orientation
        );

        // local x axis should point to world y axis now
        // move to local x axis (world y axis)
        let acc_measurement = Vector3::new(100000.0, 0.0, 0.0);
        let accel_inertial = reckoner.orientation.transform_vector(&acc_measurement);
        log_info!("accel_inertial: {}", accel_inertial);
        reckoner.update(&acc_measurement, &Vector3::zeros());
        log_info!("position: {}", reckoner.position);
        // assert!(reckoner.position.y < -0.01);
    }

    #[test]
    fn test_dead_reckoner3() {
        init_logger();

        let q_x_10 =
            UnitQuaternion::from_scaled_axis(-Vector3::<f32>::new(10f32.to_radians(), 0.0, 0.0));
        let q_y_90 =
            UnitQuaternion::from_scaled_axis(-Vector3::<f32>::new(0.0, 90f32.to_radians(), 0.0));

        log_info!("q_x_10 * q_y_90: {:?}", q_x_10 * q_y_90);
        log_info!("q_y_90 * q_x_10: {:?}", q_y_90 * q_x_10);
    }
}
