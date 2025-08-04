mod dead_reckoner;
#[cfg(test)]
mod tests;
mod welford;

use biquad::{Biquad, Coefficients, DirectForm2Transposed, Q_BUTTERWORTH_F32, ToHertz as _, Type};
use firmware_common_new::{readings::IMUData, sensor_reading::SensorReading, time::TimestampType};
use heapless::Deque;
use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

use crate::dead_reckoning::{dead_reckoner::DeadReckoner, welford::Welford3};

const SAMPLES_PER_S: usize = 500;
pub const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

const IGNITION_DETECTION_ACCELERATION_THRESHOLD: f32 = 5.0 * 9.81;
const UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

// 128KiB size budget to fit in DTCM-RAM of H743
// av frame: reference frame of the IMU ic
// rocket frame: reference frame of the rocket, z points to nose
// earth frame: inertial reference frame of the earth
pub enum RocketDeadReckoning {
    /// stable on pad, find out the orientation of the avionics relative to earth
    /// also find out imu biases
    OnPad {
        imu_data_list: Deque<IMUData, { SAMPLES_PER_S * 2 }>,
        x_acc_low_pass: DirectForm2Transposed<f32>,
        y_acc_low_pass: DirectForm2Transposed<f32>,
        z_acc_low_pass: DirectForm2Transposed<f32>,
    },

    /// first half second of powered flight, use the thrust vector to find
    /// out the orientation of the rocket relative to the avionics.
    Stage1 {
        n: usize,
        acc_welford: Welford3,
        av_orientation_reckoner: DeadReckoner,
        acc_variance: Vector3<f32>,
        gyro_variance: Vector3<f32>,
        gyro_bias: Vector3<f32>,
    },

    /// dead reckoning
    Stage2 {
        q_av_to_rocket: UnitQuaternion<f32>,
        rocket_orientation_reckoner: DeadReckoner,
        acc_variance: Vector3<f32>,
        gyro_variance: Vector3<f32>,
        gyro_bias: Vector3<f32>,
    },
    // switch to MEKF if coasting and speed < 0.9 mach
}

impl RocketDeadReckoning {
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

    pub fn update(&mut self, z: &SensorReading<impl TimestampType, IMUData>) {
        match self {
            RocketDeadReckoning::OnPad {
                imu_data_list,
                x_acc_low_pass,
                y_acc_low_pass,
                z_acc_low_pass,
            } => {
                let acc_low_passed = [
                    x_acc_low_pass.run(z.data.acc[0]),
                    y_acc_low_pass.run(z.data.acc[1]),
                    z_acc_low_pass.run(z.data.acc[2]),
                ];

                if imu_data_list.is_full() {
                    imu_data_list.pop_front().unwrap();
                }
                imu_data_list.push_back(z.data.clone()).unwrap();

                if imu_data_list.is_full() {
                    // detect ignition
                    let acc_magnitude_squared: f32 =
                        acc_low_passed.into_iter().map(|a| a * a).sum();
                    if acc_magnitude_squared
                        > IGNITION_DETECTION_ACCELERATION_THRESHOLD
                            * IGNITION_DETECTION_ACCELERATION_THRESHOLD
                    {
                        log_info!("[{}] ignition detected, to stage 1", z.timestamp_s());
                        // 2 seconds of data in imu_data_list
                        // 0s-1s: rocket stable, calculate bias, variance, and orientation between avionics to ground
                        // 1s-2s: rocket shakes due to ignition, use dead reckoning to keep track of the orientation between avionics to ground

                        enum State {
                            First {
                                acc_welford: Welford3,
                                gyro_welford: Welford3,
                            },
                            Second {
                                acc_variance: Vector3<f32>,
                                gyro_variance: Vector3<f32>,
                                gyro_bias: Vector3<f32>,
                                av_orientation_reckoner: DeadReckoner,
                            },
                        }

                        let mut state = State::First {
                            acc_welford: Welford3::new(),
                            gyro_welford: Welford3::new(),
                        };

                        for (i, imu_data) in imu_data_list.iter().enumerate() {
                            match &mut state {
                                State::First {
                                    acc_welford,
                                    gyro_welford,
                                } => {
                                    acc_welford.update(&imu_data.acc);
                                    gyro_welford.update(&imu_data.gyro);

                                    if i == SAMPLES_PER_S - 1 {
                                        // this is the gravity vector in rocket frame
                                        let gravity_vector_av_frame: Vector3<f32> =
                                            acc_welford.mean();
                                            log_info!("gravity_vector_av_frame: {}", gravity_vector_av_frame);
                                        let q_earth_to_av = quaternion_from_start_and_end_vector(
                                            &UP,
                                            &gravity_vector_av_frame,
                                        );
                                        log_info!("q_earth_to_av: {}", q_earth_to_av);
                                        let reckoner = DeadReckoner::new(q_earth_to_av);

                                        state = State::Second {
                                            acc_variance: acc_welford.variance().unwrap(),
                                            gyro_variance: gyro_welford.variance().unwrap(),
                                            gyro_bias: gyro_welford.mean(),
                                            av_orientation_reckoner: reckoner,
                                        };
                                    }
                                }
                                State::Second {
                                    av_orientation_reckoner,
                                    gyro_bias,
                                    ..
                                } => {
                                    av_orientation_reckoner
                                        .update(&imu_data.acc, &(imu_data.gyro - *gyro_bias));
                                }
                            }
                        }

                        if let State::Second {
                            acc_variance,
                            gyro_variance,
                            gyro_bias,
                            av_orientation_reckoner,
                        } = state
                        {
                            *self = RocketDeadReckoning::Stage1 {
                                n: 0,
                                acc_welford: Welford3::new(),
                                av_orientation_reckoner,
                                acc_variance,
                                gyro_variance,
                                gyro_bias,
                            };
                        } else {
                            unreachable!()
                        }
                    }
                }
            }
            RocketDeadReckoning::Stage1 {
                n,
                acc_welford,
                av_orientation_reckoner,
                acc_variance,
                gyro_variance,
                gyro_bias,
            } => {
                acc_welford.update(&z.data.acc);
                av_orientation_reckoner.update(&z.data.acc, &(z.data.gyro - *gyro_bias));

                *n += 1;
                if *n > SAMPLES_PER_S / 2 {
                    log_info!("[{}] to stage 2", z.timestamp_s());
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
                    let q_av_to_rocket = q_earth_to_rocket * q_av_to_earth; // TODO double check

                    log_info!("q_av_to_rocket: {}", q_av_to_rocket);

                    let mut rocket_orientation_reckoner = DeadReckoner::new(q_earth_to_rocket);
                    rocket_orientation_reckoner.position = av_orientation_reckoner.position;
                    rocket_orientation_reckoner.velocity = av_orientation_reckoner.velocity;

                    *self = RocketDeadReckoning::Stage2 {
                        q_av_to_rocket,
                        rocket_orientation_reckoner,
                        acc_variance: *acc_variance,
                        gyro_variance: *gyro_variance,
                        gyro_bias: *gyro_bias,
                    };
                }
            }
            RocketDeadReckoning::Stage2 {
                q_av_to_rocket,
                rocket_orientation_reckoner,
                gyro_bias,
                ..
            } => {
                let acc_rocket_frame = q_av_to_rocket.inverse_transform_vector(&z.data.acc);
                let gyro_rocket_frame =
                    q_av_to_rocket.inverse_transform_vector(&(z.data.gyro - *gyro_bias));

                rocket_orientation_reckoner.update(&acc_rocket_frame, &gyro_rocket_frame);
            }
        }
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

        log_info!("{}", size_of::<RocketDeadReckoning>())
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
