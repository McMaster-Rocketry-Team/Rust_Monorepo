use nalgebra::{UnitQuaternion, Vector2};

use crate::state_estimator2::{
    FlightProfile, Measurement,
    ascent::{
        altitude_estimator::AltitudeEstimator, orientation_estimator::OrientationEstimator,
        velocity_estimator::VelocityEstimator,
    },
};

mod altitude_estimator;
mod altitude_kf;
mod dead_reckoner;
mod orientation_estimator;
#[cfg(test)]
mod tests;
mod velocity_estimator;
mod welford;

pub struct AscentStateEstimator {
    orientation_estimator: OrientationEstimator,
    altitude_estimator: AltitudeEstimator,
    velocity_estimator: Option<VelocityEstimator>,
    profile: FlightProfile,
}

impl AscentStateEstimator {
    pub fn new(flight_profile: FlightProfile) -> Self {
        Self {
            orientation_estimator: OrientationEstimator::new(flight_profile.clone()),
            altitude_estimator: AltitudeEstimator::new(flight_profile.clone()),
            velocity_estimator: None,
            profile: flight_profile,
        }
    }

    pub fn update(&mut self, z_imu_frame: &Measurement) -> bool {
        self.orientation_estimator.update(z_imu_frame);

        let acc_vertical = match &self.orientation_estimator {
            OrientationEstimator::OnPad { .. } => 0.0f32,
            OrientationEstimator::Stage1 {
                av_orientation_reckoner,
                ..
            }
            | OrientationEstimator::Stage2 {
                av_orientation_reckoner,
                ..
            } => {
                av_orientation_reckoner
                    .orientation
                    .transform_vector(&z_imu_frame.acceleration())
                    .z
                    - 9.81
            }
        };
        self.altitude_estimator.update(acc_vertical, z_imu_frame);

        let vertical_velocity = self.altitude_estimator.velocity();
        if let Some(tilt) = self.orientation_estimator.tilt()
            && vertical_velocity > 1.0
        {
            let velocity_direction_2d = Vector2::new(tilt.sin(), tilt.cos());
            let velocity_2d = velocity_direction_2d * (vertical_velocity / velocity_direction_2d.y);
            plot_add_value!("velocity 2d x", velocity_2d.x);
            if let Some(velocity_estimator) = &mut self.velocity_estimator {
                velocity_estimator.update(velocity_2d);
            } else {
                self.velocity_estimator = Some(VelocityEstimator::new(
                    tilt,
                    velocity_2d.magnitude(),
                    0.0,
                    // TODO
                    0.1,
                    0.1,
                    1.0,
                ))
            }
        }

        self.altitude_estimator.is_apogee()
    }

    pub fn tilt_and_velocity(&self) -> Option<(f32, Vector2<f32>)> {
        if let Some(tilt) = self.orientation_estimator.tilt()
            && let Some(velocity_estimator) = &self.velocity_estimator
        {
            Some((tilt, velocity_estimator.state().0))
        } else {
            None
        }
    }

    pub fn rocket_orientation(&self) -> Option<UnitQuaternion<f32>> {
        match &self.orientation_estimator {
            OrientationEstimator::Stage2 {
                q_av_to_rocket,
                av_orientation_reckoner,
                ..
            } => Some(av_orientation_reckoner.orientation * *q_av_to_rocket),
            _ => None,
        }
    }

    pub fn altitude_agl(&self) -> f32 {
        self.altitude_estimator.altitude_agl()
    }
}
