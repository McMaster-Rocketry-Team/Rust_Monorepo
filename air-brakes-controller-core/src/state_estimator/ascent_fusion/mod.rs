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

pub enum AscentFusionStateEstimator {
    Bootstrap {
        estimator: BootstrapStateEstimator,
        constants: RocketConstants,
    },
    Ready {
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
                        rocket_orientation_reckoner,
                        acc_variance,
                        gyro_variance,
                        alt_variance,
                        last_acc_rocket_frame,
                        last_gyro_rocket_frame,
                        ..
                    } = estimator
                {
                    let rocket_orientation = rocket_orientation_reckoner.orientation;
                    *self = Self::Ready {
                        estimator: MekfStateEstimator::new(
                            rocket_orientation,
                            State::new(
                                &Vector3::zeros(),
                                &rocket_orientation.inverse_transform_vector(last_acc_rocket_frame),
                                &rocket_orientation_reckoner.velocity,
                                &rocket_orientation
                                    .inverse_transform_vector(last_gyro_rocket_frame),
                                rocket_orientation_reckoner.position.z,
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
                    };
                }
            }
            Self::Ready {
                estimator,
                q_av_to_rocket,
                gyro_bias,
            } => {
                let acc_rocket_frame =
                    q_av_to_rocket.inverse_transform_vector(&z_imu_frame.acceleration());
                let gyro_rocket_frame = q_av_to_rocket
                    .inverse_transform_vector(&(z_imu_frame.angular_velocity() - *gyro_bias));

                let z_rocket_frame = Measurement::new(&acc_rocket_frame, &gyro_rocket_frame, z_imu_frame.altitude_asl());
                estimator.predict(airbrakes_ext);
                estimator.update(z_rocket_frame);
            }
        }
    }
}
