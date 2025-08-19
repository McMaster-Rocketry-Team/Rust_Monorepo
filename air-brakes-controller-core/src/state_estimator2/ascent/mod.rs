use biquad::{
    Biquad as _, Coefficients, DirectForm2Transposed, Q_BUTTERWORTH_F32, ToHertz as _, Type,
};
use heapless::Deque;
use micromath::F32Ext;
use nalgebra::{UnitQuaternion, UnitVector3, Vector1, Vector2, Vector3};

use crate::{
    state_estimator2::{
        FlightProfile, Measurement, SAMPLES_PER_S,
        ascent::{
            dead_reckoner::DeadReckoner,
            velocity_estimator::{MeasNoiseStd, ProcessNoiseStd, VelocityEstimator},
            welford::Welford,
        },
    },
    utils::approximate_speed_of_sound,
};

mod dead_reckoner;
#[cfg(test)]
mod tests;
mod velocity_estimator;
mod welford;

const UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaroLockOutState {
    BeforeLockOut,
    LockOut,
    AfterLockOut,
}

// av frame: reference frame of the IMU ic
// rocket frame: reference frame of the rocket, z points to nose
// earth frame: inertial reference frame of the earth
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum AscentStateEstimator {
    /// stable on pad, find out the orientation of the avionics relative to earth
    /// also find out imu biases
    OnPad {
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        imu_data_list: Deque<Measurement, { SAMPLES_PER_S * 2 }>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        x_acc_low_pass: DirectForm2Transposed<f32>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        y_acc_low_pass: DirectForm2Transposed<f32>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        z_acc_low_pass: DirectForm2Transposed<f32>,
        flight_profile: FlightProfile,
    },

    /// first half second of powered flight, use the thrust vector to find
    /// out the orientation of the rocket relative to the avionics.
    Stage1 {
        n: usize,
        acc_welford: Welford<3>,
        pad_av_orientation: UnitQuaternion<f32>,
        av_orientation_reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        alt_variance: f32,
        launch_pad_altitude_asl: f32,
    },

    /// dead reckoning until ap
    Stage2 {
        q_av_to_rocket: UnitQuaternion<f32>,
        av_orientation_reckoner: DeadReckoner,
        gyro_bias: Vector3<f32>,
        velocity_estimator: VelocityEstimator,
        lock_out_state: BaroLockOutState,
        alt_variance: f32,
        launch_pad_altitude_asl: f32,
        is_coasting: bool,
    },

    Apogee {
        altitude_asl: f32,
        alt_variance: f32,
        launch_pad_altitude_asl: f32,
    },
}

impl AscentStateEstimator {
    pub(crate) fn new(flight_profile: FlightProfile) -> Self {
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
            flight_profile,
        }
    }

    pub(crate) fn update(&mut self, z_imu_frame: &Measurement) {
        let acc = z_imu_frame.acceleration();
        let gyro = z_imu_frame.angular_velocity();
        match self {
            Self::OnPad {
                imu_data_list,
                x_acc_low_pass,
                y_acc_low_pass,
                z_acc_low_pass,
                flight_profile,
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
                    // if acc_magnitude_squared.sqrt() > 11.0 {
                    //     log_info!("magnitude: {}", acc_magnitude_squared.sqrt());
                    // }

                    if acc_magnitude_squared
                        > flight_profile.ignition_detection_acc_threshold
                            * flight_profile.ignition_detection_acc_threshold
                    {
                        log_info!(
                            "[{}] ignition detected, to stage 1",
                            plot_get_time_s!()
                        );
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
                                gyro_bias: Vector3<f32>,
                                pad_av_orientation: UnitQuaternion<f32>,
                                av_orientation_reckoner: DeadReckoner,
                                alt_variance: f32,
                                launch_pad_altitude_asl: f32,
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
                                    alt_asl_welford.update(&Vector1::new(past_z.altitude_asl()));

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
                                        let launch_pad_altitude_asl = alt_asl_welford.mean()[0];
                                        reckoner.position.z = launch_pad_altitude_asl;

                                        state = State::Second {
                                            gyro_bias: gyro_welford.mean(),
                                            pad_av_orientation: q_earth_to_av,
                                            av_orientation_reckoner: reckoner,
                                            alt_variance: alt_asl_welford.variance().unwrap()[0],
                                            launch_pad_altitude_asl,
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
                            gyro_bias,
                            pad_av_orientation,
                            av_orientation_reckoner,
                            alt_variance,
                            launch_pad_altitude_asl,
                        } = state
                        {
                            log_info!("[{}] go to stage 1, {:?}", plot_get_time_s!(), av_orientation_reckoner);
                            *self = Self::Stage1 {
                                n: 0,
                                acc_welford: Welford::<3>::new(),
                                pad_av_orientation,
                                av_orientation_reckoner,
                                gyro_bias,
                                alt_variance,
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
                pad_av_orientation,
                av_orientation_reckoner,
                gyro_bias,
                alt_variance,
                launch_pad_altitude_asl,
            } => {
                acc_welford.update(&acc);
                av_orientation_reckoner.update(&acc, &(gyro - *gyro_bias));

                *n += 1;
                if *n > SAMPLES_PER_S / 2 {
                    log_info!("[{}] to stage 2", plot_get_time_s!());
                    let avg_acc_av_frame = acc_welford.mean();
                    let avg_acc_earth_frame =
                        pad_av_orientation.transform_vector(&avg_acc_av_frame);

                    let launch_angle_deg = UP.angle(&avg_acc_earth_frame).to_degrees();
                    log_info!("launch angle degree: {}", launch_angle_deg);

                    let q_earth_to_rocket =
                        quaternion_from_start_and_end_vector(&avg_acc_earth_frame, &UP);
                    log_info!("q_earth_to_rocket: {}", q_earth_to_rocket);

                    let q_av_to_earth = pad_av_orientation.inverse();
                    let q_av_to_rocket = q_av_to_earth * q_earth_to_rocket;

                    *self = Self::Stage2 {
                        q_av_to_rocket,
                        av_orientation_reckoner: av_orientation_reckoner.clone(),
                        gyro_bias: *gyro_bias,
                        lock_out_state: BaroLockOutState::BeforeLockOut,
                        velocity_estimator: VelocityEstimator::new(
                            av_orientation_reckoner.position.z,
                            av_orientation_reckoner.velocity.magnitude(),
                            tilt(&q_av_to_rocket, &av_orientation_reckoner),
                            // TODO tune
                            ProcessNoiseStd {
                                z: 0.15,
                                s: 8.0,
                                theta: 0.8f32.to_radians(),
                                omega: 3.0f32.to_radians(),
                                b: 0.07f32.to_radians(),
                            },
                            MeasNoiseStd {
                                tilt: 2.5f32.to_radians(),
                                alt: 3.0,
                            },
                        ),
                        alt_variance: *alt_variance,
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        is_coasting: false,
                    };

                    log_info!("state: {:?}", *self);
                }
            }
            Self::Stage2 {
                av_orientation_reckoner,
                gyro_bias,
                velocity_estimator,
                q_av_to_rocket,
                lock_out_state,
                alt_variance,
                launch_pad_altitude_asl,
                is_coasting,
                ..
            } => {
                av_orientation_reckoner.update(&acc, &(gyro - *gyro_bias));

                if !*is_coasting {
                    let speed_acc_angle = av_orientation_reckoner
                        .acceleration
                        .angle(&av_orientation_reckoner.velocity);
                    if speed_acc_angle > 90f32.to_radians() {
                        *is_coasting = true;
                    }
                }

                let mut horizontal_acceleration = Vector2::new(
                    av_orientation_reckoner.acceleration.x,
                    av_orientation_reckoner.acceleration.y,
                )
                .magnitude();
                if *is_coasting {
                    horizontal_acceleration = -horizontal_acceleration;
                }
                velocity_estimator.predict(
                    av_orientation_reckoner.acceleration.z,
                    horizontal_acceleration,
                );
                if *lock_out_state == BaroLockOutState::LockOut {
                    velocity_estimator
                        .update(tilt(&q_av_to_rocket, &av_orientation_reckoner), None);
                } else {
                    velocity_estimator.update(
                        tilt(&q_av_to_rocket, &av_orientation_reckoner),
                        Some(z_imu_frame.altitude_asl()),
                    );
                }

                match lock_out_state {
                    BaroLockOutState::BeforeLockOut => {
                        let speed_of_sound =
                            approximate_speed_of_sound(velocity_estimator.altitude_asl());
                        if velocity_estimator.v_vertical().abs() > 0.9 * speed_of_sound {
                            log_info!("baro lock out");
                            *lock_out_state = BaroLockOutState::LockOut;
                        }
                    }
                    BaroLockOutState::LockOut => {
                        let speed_of_sound =
                            approximate_speed_of_sound(velocity_estimator.altitude_asl());
                        if velocity_estimator.v_vertical().abs() < 0.85 * speed_of_sound {
                            log_info!("baro lock out finished");
                            *lock_out_state = BaroLockOutState::AfterLockOut;
                            velocity_estimator.constraints_enabled = true;
                            // TODO also update process noise
                        }
                    }
                    BaroLockOutState::AfterLockOut => {}
                }

                if velocity_estimator.v_vertical() < 1.0 {
                    log_info!("[{}] apogee detected", plot_get_time_s!());
                    *self = Self::Apogee {
                        alt_variance: *alt_variance,
                        altitude_asl: velocity_estimator.altitude_asl(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    }
                }
            }
            Self::Apogee { .. } => {}
        }
    }

    pub fn is_coasting(&self) -> bool {
        match self {
            Self::OnPad { .. } | Self::Stage1 { .. } => false,
            Self::Stage2 { is_coasting, .. } => *is_coasting,
            Self::Apogee { .. } => true,
        }
    }

    pub fn altitude_asl(&self) -> Option<f32> {
        match self {
            Self::OnPad { .. } => None,
            Self::Stage1 {
                av_orientation_reckoner,
                ..
            } => Some(av_orientation_reckoner.position.z),
            Self::Stage2 {
                velocity_estimator, ..
            } => Some(velocity_estimator.altitude_asl()),
            Self::Apogee { altitude_asl, .. } => Some(*altitude_asl),
        }
    }

    pub fn launch_pad_altitude_asl(&self) -> Option<f32> {
        match self {
            Self::OnPad { .. } => None,
            Self::Stage1 {
                launch_pad_altitude_asl,
                ..
            }
            | Self::Stage2 {
                launch_pad_altitude_asl,
                ..
            }
            | Self::Apogee {
                launch_pad_altitude_asl,
                ..
            } => Some(*launch_pad_altitude_asl),
        }
    }

    pub fn rocket_orientation(&self) -> Option<UnitQuaternion<f32>> {
        match self {
            Self::Stage2 {
                q_av_to_rocket,
                av_orientation_reckoner,
                ..
            } => Some(av_orientation_reckoner.orientation * *q_av_to_rocket),
            _ => None,
        }
    }

    /// x and y always positive
    pub fn velocity(&self) -> Option<Vector2<f32>> {
        match self {
            Self::Stage2 {
                velocity_estimator, ..
            } => Some(Vector2::new(
                velocity_estimator.v_horizontal(),
                velocity_estimator.v_vertical(),
            )),
            _ => None,
        }
    }
}

fn tilt(q_av_to_rocket: &UnitQuaternion<f32>, av_orientation_reckoner: &DeadReckoner) -> f32 {
    let rocket_orientation = av_orientation_reckoner.orientation * *q_av_to_rocket;
    let up = Vector3::new(0f32, 0f32, 1f32);

    let velocity_direction = rocket_orientation.transform_vector(&up);
    up.angle(&velocity_direction)
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

    if angle.to_degrees() < 0.05 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_axis_angle(&axis, angle)
    }
}
