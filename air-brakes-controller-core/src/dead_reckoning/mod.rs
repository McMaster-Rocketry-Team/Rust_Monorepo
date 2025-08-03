mod welford;
mod dead_reckoner;

use biquad::{Biquad, Coefficients, DirectForm2Transposed, Q_BUTTERWORTH_F32, ToHertz as _, Type};
use firmware_common_new::readings::IMUData;
use heapless::Deque;
use nalgebra::{Quaternion, UnitQuaternion, UnitVector3, Vector3};

use crate::dead_reckoning::welford::Welford3;

const SAMPLES_PER_S: usize = 500;
pub const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

const IGNITION_DETECTION_ACCELERATION_THRESHOLD: f32 = 5.0 * 9.81;

// 128KiB size budget to fit in DTCM-RAM of H743
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
    /// In a real launch, there will be some vibrations right after
    /// ignition, which need to be filtered out
    Stage1 {
        imu_data_list: Deque<IMUData, { SAMPLES_PER_S / 2 }>,
        acc_variance: [f32; 3],
        gyro_variance: [f32; 3],
        gyro_bias: [f32; 3],
    },

    /// dead reckoning
    Stage2 { gyro_bias: [f32; 3] },
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

    pub fn update(&mut self, z: IMUData) {
        match self {
            RocketDeadReckoning::OnPad {
                imu_data_list,
                x_acc_low_pass,
                y_acc_low_pass,
                z_acc_low_pass,
            } => {
                let acc_low_passed = [
                    x_acc_low_pass.run(z.acc[0]),
                    y_acc_low_pass.run(z.acc[1]),
                    z_acc_low_pass.run(z.acc[2]),
                ];

                if imu_data_list.is_full() {
                    imu_data_list.pop_back().unwrap();
                }
                imu_data_list.push_front(z).unwrap();

                if imu_data_list.is_full() {
                    // detect ignition
                    let acc_magnitude_squared: f32 =
                        acc_low_passed.into_iter().map(|a| a * a).sum();
                    if acc_magnitude_squared
                        > IGNITION_DETECTION_ACCELERATION_THRESHOLD
                            * IGNITION_DETECTION_ACCELERATION_THRESHOLD
                    {
                        // 2 seconds of data in imu_data_list
                        // 0s-1s: rocket stable, calculate bias, variance, and orientation between avionics to ground
                        // 1s-2s: rocket shakes due to ignition, use dead reckoning to keep track of the orientation between avionics to ground

                        enum State {
                            First {
                                acc_welford: Welford3,
                                gyro_welford: Welford3,
                            },
                            Second {
                                acc_variance: [f32; 3],
                                gyro_variance: [f32; 3],
                                gyro_bias: [f32; 3],
                                q_imu_to_inertial: UnitQuaternion<f32>,
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
                                        let gravity_vector_rocket_frame: Vector3<f32> =
                                            acc_welford.mean().into();
                                        let up_vector_inertial_frame: Vector3<f32> =
                                            Vector3::new(0.0, 0.0, 1.0);

                                        let q_imu_to_inertial = quaternion_from_start_and_end_vector(
                                            gravity_vector_rocket_frame,
                                            up_vector_inertial_frame,
                                        );

                                        state = State::Second {
                                            acc_variance: acc_welford.variance().unwrap(),
                                            gyro_variance: gyro_welford.variance().unwrap(),
                                            gyro_bias: gyro_welford.mean(),
                                            q_imu_to_inertial,
                                        };
                                    }
                                }
                                State::Second {
                                    acc_variance,
                                    gyro_variance,
                                    gyro_bias,
                                    q_imu_to_inertial,
                                } => {
                                    
                                },
                            }
                        }

                        // calculate biases

                        // calculate orientation of the avionics relative to earth

                        // to stage 1
                    }
                }
            }
            RocketDeadReckoning::Stage1 { .. } => todo!(),
            RocketDeadReckoning::Stage2 { .. } => todo!(),
        }
    }
}

fn quaternion_from_start_and_end_vector(
    start: Vector3<f32>,
    end: Vector3<f32>,
) -> UnitQuaternion<f32> {
    let start = start.normalize();
    let end = end.normalize();

    let axis = UnitVector3::new_unchecked(end.cross(&start));
    let angle = end.angle(&start);

    UnitQuaternion::from_axis_angle(&axis, angle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::init_logger;

    #[test]
    fn struct_size() {
        init_logger();

        log_info!("{}", size_of::<RocketDeadReckoning>())
    }
}
