use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    RocketConstants,
    state_estimator::{
        Measurement,
        ascent_fusion::{
            bootstrap::BootstrapStateEstimator,
            mekf::{MekfStateEstimator, State},
        },
    },
};

mod bootstrap;
mod mekf;
#[cfg(test)]
mod tests;

pub enum AscentFusionStateEstimator {
    Bootstrap {
        estimator: BootstrapStateEstimator,
        constants: RocketConstants,
    },
    Ready {
        launch_pad_altitude_asl: f32,
        estimator: MekfStateEstimator,
        q_av_to_rocket: UnitQuaternion<f32>,
        gyro_bias: Vector3<f32>,
    },
}

impl AscentFusionStateEstimator {
    pub fn new(constants: RocketConstants) -> Self {
        Self::Bootstrap {
            estimator: BootstrapStateEstimator::new(),
            constants,
        }
    }

    pub fn update(&mut self, airbrakes_ext: f32, z_imu_frame: &Measurement) {
        match self {
            Self::Bootstrap {
                estimator,
                constants,
            } => {
                estimator.update(z_imu_frame);

                if estimator.should_switch_to_mekf()
                    && let BootstrapStateEstimator::Stage2 {
                        q_av_to_rocket,
                        gyro_bias,
                        av_orientation_reckoner,
                        acc_variance,
                        gyro_variance,
                        alt_variance,
                        last_gyro_imu_frame_unbiased,
                        launch_pad_altitude_asl,
                        ..
                    } = estimator
                {
                    log_info!("[{}] switch to mekf", plot_get_time_s!());
                    let av_orientation = av_orientation_reckoner.orientation;
                    let rocket_orientation = av_orientation * *q_av_to_rocket;
                    *self = Self::Ready {
                        estimator: MekfStateEstimator::new(
                            rocket_orientation,
                            State::new(
                                &Vector3::zeros(),
                                &av_orientation_reckoner.velocity,
                                &av_orientation.transform_vector(last_gyro_imu_frame_unbiased),
                                av_orientation_reckoner.position.z,
                                constants.initial_sideways_moment_co,
                                &constants.initial_front_cd.into(),
                            ),
                            *acc_variance,
                            *gyro_variance,
                            *alt_variance,
                            constants.clone(),
                        ),
                        q_av_to_rocket: *q_av_to_rocket,
                        gyro_bias: *gyro_bias,
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::Ready {
                estimator,
                q_av_to_rocket,
                gyro_bias,
                ..
            } => {
                let av_orientation = estimator.orientation * q_av_to_rocket.inverse();
                let acc_earth_frame = av_orientation.transform_vector(&z_imu_frame.acceleration());
                let gyro_earth_frame =
                    av_orientation.transform_vector(&(z_imu_frame.angular_velocity() - *gyro_bias));

                let z_earth_frame = Measurement::new(
                    &acc_earth_frame,
                    &gyro_earth_frame,
                    z_imu_frame.altitude_asl(),
                );
                estimator.predict(airbrakes_ext);
                estimator.update(airbrakes_ext, z_earth_frame);
            }
        }
    }

    pub fn altitude_agl(&self) -> f32 {
        match self {
            Self::Bootstrap {
                estimator: BootstrapStateEstimator::OnPad { .. },
                ..
            } => 0.0,
            Self::Bootstrap {
                estimator:
                    BootstrapStateEstimator::Stage1 {
                        av_orientation_reckoner,
                        ..
                    },
                ..
            } => av_orientation_reckoner.position.z,
            Self::Bootstrap {
                estimator:
                    BootstrapStateEstimator::Stage2 {
                        av_orientation_reckoner,
                        ..
                    },
                ..
            } => av_orientation_reckoner.position.z,
            Self::Ready {
                estimator,
                launch_pad_altitude_asl,
                ..
            } => estimator.state.altitude_asl() - launch_pad_altitude_asl,
        }
    }

    pub fn rocket_orientation(&self) -> Option<UnitQuaternion<f32>> {
        match self {
            Self::Bootstrap {
                estimator: BootstrapStateEstimator::OnPad { .. },
                ..
            } => None,
            Self::Bootstrap {
                estimator: BootstrapStateEstimator::Stage1 { .. },
                ..
            } => None,
            Self::Bootstrap {
                estimator:
                    BootstrapStateEstimator::Stage2 {
                        q_av_to_rocket,
                        av_orientation_reckoner,
                        ..
                    },
                ..
            } => Some(av_orientation_reckoner.orientation * *q_av_to_rocket),
            Self::Ready { estimator, .. } => Some(estimator.orientation),
        }
    }
}
