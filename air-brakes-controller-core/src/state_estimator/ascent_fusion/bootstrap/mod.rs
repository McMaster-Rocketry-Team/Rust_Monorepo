mod dead_reckoner;
#[cfg(test)]
mod tests;

use biquad::{Biquad, Coefficients, DirectForm2Transposed, Q_BUTTERWORTH_F32, ToHertz as _, Type};
use heapless::Deque;
use nalgebra::{UnitQuaternion, UnitVector3, Vector1, Vector3};

use crate::{
    state_estimator::{
        Measurement, SAMPLES_PER_S, ascent_fusion::bootstrap::dead_reckoner::DeadReckoner,
        welford::Welford,
    },
    utils::approximate_speed_of_sound,
};

const IGNITION_DETECTION_ACCELERATION_THRESHOLD: f32 = 5.0 * 9.81;
const UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

// av frame: reference frame of the IMU ic
// rocket frame: reference frame of the rocket, z points to nose
// earth frame: inertial reference frame of the earth
pub enum BootstrapStateEstimator {
    /// stable on pad, find out the orientation of the avionics relative to earth
    /// also find out imu biases
    OnPad {
        imu_data_list: Deque<Measurement, { SAMPLES_PER_S * 2 }>,
        x_acc_low_pass: DirectForm2Transposed<f32>,
        y_acc_low_pass: DirectForm2Transposed<f32>,
        z_acc_low_pass: DirectForm2Transposed<f32>,
    },

    /// first half second of powered flight, use the thrust vector to find
    /// out the orientation of the rocket relative to the avionics.
    Stage1 {
        n: usize,
        acc_welford: Welford<3>,
        av_orientation_reckoner: DeadReckoner,
        alt_variance: f32,
        acc_variance: f32,
        gyro_variance: f32,
        gyro_bias: Vector3<f32>,
        launch_pad_altitude_asl: f32,
    },

    /// dead reckoning
    Stage2 {
        q_av_to_rocket: UnitQuaternion<f32>,
        rocket_orientation_reckoner: DeadReckoner,
        last_acc_rocket_frame: Vector3<f32>,
        last_gyro_rocket_frame: Vector3<f32>,
        alt_variance: f32,
        acc_variance: f32,
        gyro_variance: f32,
        gyro_bias: Vector3<f32>,
        launch_pad_altitude_asl: f32,
    },
    // switch to MEKF if coasting and speed < 0.8 mach
}

impl BootstrapStateEstimator {
    pub fn new() -> Self {
        let acc_low_pass_coeff = Coefficients::<f32>::from_params(
            Type::LowPass,
            (SAMPLES_PER_S as f32).hz(),
            10f32.hz(),
            Q_BUTTERWORTH_F32,
        )
        .unwrap();
        Self::OnPad {
            imu_data_list: Deque::new(),
            x_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
            y_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
            z_acc_low_pass: DirectForm2Transposed::<f32>::new(acc_low_pass_coeff),
        }
    }

    pub fn update(&mut self, z_imu_frame: &Measurement) {
        let acc = z_imu_frame.acceleration();
        let gyro = z_imu_frame.angular_velocity();
        match self {
            Self::OnPad {
                imu_data_list,
                x_acc_low_pass,
                y_acc_low_pass,
                z_acc_low_pass,
            } => {
                let acc_low_passed = [
                    x_acc_low_pass.run(acc[0]),
                    y_acc_low_pass.run(acc[1]),
                    z_acc_low_pass.run(acc[2]),
                ];

                if imu_data_list.is_full() {
                    imu_data_list.pop_front().unwrap();
                }
                imu_data_list.push_back(z_imu_frame.clone()).unwrap();

                if imu_data_list.is_full() {
                    // detect ignition
                    let acc_magnitude_squared: f32 =
                        acc_low_passed.into_iter().map(|a| a * a).sum();
                    if acc_magnitude_squared
                        > IGNITION_DETECTION_ACCELERATION_THRESHOLD
                            * IGNITION_DETECTION_ACCELERATION_THRESHOLD
                    {
                        // log_info!(
                        //     "[{}] ignition detected, to stage 1",
                        //     z_imu_frame.timestamp_s()
                        // );
                        // 2 seconds of data in imu_data_list
                        // 0s-1s: rocket stable, calculate bias, variance, and orientation between avionics to ground
                        // 1s-2s: rocket shakes due to ignition, use dead reckoning to keep track of the orientation between avionics to ground

                        enum State {
                            First {
                                acc_welford: Welford<3>,
                                gyro_welford: Welford<3>,
                                alt_asl_welford: Welford<1>,
                            },
                            Second {
                                alt_variance: f32, 
                                acc_variance: f32,
                                gyro_variance: f32,
                                gyro_bias: Vector3<f32>,
                                av_orientation_reckoner: DeadReckoner,
                                launch_pad_altitude_asl:f32,
                            },
                        }

                        let mut state = State::First {
                            acc_welford: Welford::<3>::new(),
                            gyro_welford: Welford::<3>::new(),
                            alt_asl_welford: Welford::<1>::new(),
                        };

                        for (i, past_z) in imu_data_list.iter().enumerate() {
                            match &mut state {
                                State::First {
                                    acc_welford,
                                    gyro_welford,
                                    alt_asl_welford,
                                } => {
                                    acc_welford.update(&past_z.acceleration());
                                    gyro_welford.update(&past_z.angular_velocity());
                                    alt_asl_welford
                                        .update(&Vector1::new(past_z.altitude_asl()));

                                    if i == SAMPLES_PER_S - 1 {
                                        // this is the gravity vector in rocket frame
                                        let gravity_vector_av_frame: Vector3<f32> =
                                            acc_welford.mean();
                                        log_info!(
                                            "gravity_vector_av_frame: {}",
                                            gravity_vector_av_frame
                                        );
                                        let q_earth_to_av = quaternion_from_start_and_end_vector(
                                            &UP,
                                            &gravity_vector_av_frame,
                                        );
                                        log_info!("q_earth_to_av: {}", q_earth_to_av);
                                        let mut reckoner = DeadReckoner::new(q_earth_to_av);
                                        let launch_pad_altitude_asl =  alt_asl_welford.mean()[0];
                                        reckoner.position.z =launch_pad_altitude_asl;

                                        state = State::Second {
                                            alt_variance: alt_asl_welford.variance().unwrap()[0],
                                            acc_variance: acc_welford.variance_magnitude().unwrap(),
                                            gyro_variance: gyro_welford.variance_magnitude().unwrap(),
                                            gyro_bias: gyro_welford.mean(),
                                            av_orientation_reckoner: reckoner,
                                            launch_pad_altitude_asl
                                        };
                                    }
                                }
                                State::Second {
                                    av_orientation_reckoner,
                                    gyro_bias,
                                    ..
                                } => {
                                    av_orientation_reckoner.update(
                                        &past_z.acceleration(),
                                        &(past_z.angular_velocity() - *gyro_bias),
                                    );
                                }
                            }
                        }

                        if let State::Second {
                            alt_variance,
                            acc_variance,
                            gyro_variance,
                            gyro_bias,
                            av_orientation_reckoner,
                            launch_pad_altitude_asl,
                        } = state
                        {
                            *self = Self::Stage1 {
                                n: 0,
                                acc_welford: Welford::<3>::new(),
                                av_orientation_reckoner,
                                alt_variance,
                                acc_variance,
                                gyro_variance,
                                gyro_bias,
                                launch_pad_altitude_asl,
                            };
                        } else {
                            unreachable!()
                        }
                    }
                }
            }
            Self::Stage1 {
                n,
                acc_welford,
                av_orientation_reckoner,
                alt_variance,
                acc_variance,
                gyro_variance,
                gyro_bias,
                launch_pad_altitude_asl,
            } => {
                acc_welford.update(&acc);
                av_orientation_reckoner.update(&acc, &(gyro - *gyro_bias));

                *n += 1;
                if *n > SAMPLES_PER_S / 2 {
                    // log_info!("[{}] to stage 2", z_imu_frame.timestamp_s());
                    let avg_acc_av_frame = acc_welford.mean();
                    let avg_acc_earth_frame = av_orientation_reckoner
                        .orientation
                        .transform_vector(&avg_acc_av_frame);

                    let launch_angle_deg = UP.angle(&avg_acc_earth_frame).to_degrees();
                    log_info!("launch angle degree: {}", launch_angle_deg);

                    let q_earth_to_rocket =
                        quaternion_from_start_and_end_vector(&avg_acc_earth_frame, &UP);
                    log_info!("q_earth_to_rocket: {}", q_earth_to_rocket);

                    let q_av_to_earth = av_orientation_reckoner.orientation.inverse();
                    let q_av_to_rocket = q_earth_to_rocket * q_av_to_earth;

                    log_info!("q_av_to_rocket: {}", q_av_to_rocket);

                    let mut rocket_orientation_reckoner = DeadReckoner::new(q_earth_to_rocket);
                    rocket_orientation_reckoner.position = av_orientation_reckoner.position;
                    rocket_orientation_reckoner.velocity = av_orientation_reckoner.velocity;

                    *self = Self::Stage2 {
                        q_av_to_rocket,
                        rocket_orientation_reckoner,
                        alt_variance: *alt_variance,
                        acc_variance: *acc_variance,
                        gyro_variance: *gyro_variance,
                        gyro_bias: *gyro_bias,
                        last_acc_rocket_frame: q_av_to_rocket.inverse_transform_vector(&acc),
                        last_gyro_rocket_frame: q_av_to_rocket.inverse_transform_vector(&gyro),
                        launch_pad_altitude_asl:*launch_pad_altitude_asl,
                    };
                }
            }
            Self::Stage2 {
                q_av_to_rocket,
                rocket_orientation_reckoner,
                gyro_bias,
                last_acc_rocket_frame,
                last_gyro_rocket_frame,
                ..
            } => {
                let acc_rocket_frame = q_av_to_rocket.inverse_transform_vector(&acc);
                let gyro_rocket_frame =
                    q_av_to_rocket.inverse_transform_vector(&(gyro - *gyro_bias));

                rocket_orientation_reckoner.update(&acc_rocket_frame, &gyro_rocket_frame);
                *last_acc_rocket_frame = acc_rocket_frame;
                *last_gyro_rocket_frame = gyro_rocket_frame;
            }
        }
    }

    pub fn should_switch_to_mekf(&self) -> bool {
        if let Self::Stage2 {
            last_acc_rocket_frame,
            rocket_orientation_reckoner,
            ..
        } = self
        {
            let speed_of_sound = approximate_speed_of_sound(rocket_orientation_reckoner.position.z);
            if last_acc_rocket_frame.z < 0.0
                && rocket_orientation_reckoner.velocity.magnitude_squared()
                    < (0.8 * speed_of_sound * 0.8 * speed_of_sound)
            {
                return true;
            }
        }

        false
    }
}

/// returns a passive rotation quaternion that would rotate start vector to end vector
fn quaternion_from_start_and_end_vector(
    start: &Vector3<f32>,
    end: &Vector3<f32>,
) -> UnitQuaternion<f32> {
    let start = start.normalize();
    let end = end.normalize();

    let axis = UnitVector3::new_normalize(end.cross(&start));
    let angle = end.angle(&start);

    UnitQuaternion::from_axis_angle(&axis, angle)
}

#[cfg(test)]
mod tests2 {
    use super::*;
    use crate::tests::init_logger;

    #[test]
    fn struct_size() {
        init_logger();

        log_info!("{}", size_of::<BootstrapStateEstimator>())
    }

    #[test]
    fn test_quaternion_from_start_and_end_vector() {
        init_logger();

        let start = Vector3::new(0.0, 0.0, 1.0);
        let end = Vector3::new(1.0, 0.0, 1.0).normalize();

        let q_start_to_end = quaternion_from_start_and_end_vector(&start, &end);

        let end2 = q_start_to_end.inverse_transform_vector(&start);

        log_info!("{}, {}", end, end2)
    }
}
