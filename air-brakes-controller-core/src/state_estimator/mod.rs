use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use nalgebra::{SVector, Vector3};

use crate::{state_estimator::{ascent_fusion::AscentFusionStateEstimator, baro::BaroStateEstimator}, RocketConstants};

mod ascent_fusion;
mod baro;
mod welford;

const SAMPLES_PER_S: usize = 500;
const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

// 128KiB size budget to fit in DTCM-RAM of H743
pub enum RocketStateEstimator {
    Ascent {
        baro_estimator: BaroStateEstimator,
        fusion_estimator: AscentFusionStateEstimator,
    },
    Descent {
        baro_estimator: BaroStateEstimator,
    },
}

impl RocketStateEstimator {
    pub fn new(profile: FlightProfile, constants: RocketConstants) -> Self {
        Self::Ascent {
            baro_estimator: BaroStateEstimator::new(profile),
            fusion_estimator: AscentFusionStateEstimator::new(constants),
        }
    }

    pub fn update(
        &mut self,
        airbrakes_ext: f32,
        z_imu_frame: &Measurement,
    ) -> Option<PyroSelect> {
        match self {
            Self::Ascent {
                baro_estimator,
                fusion_estimator,
            } => {
                fusion_estimator.update(airbrakes_ext, z_imu_frame);
                let result = baro_estimator.update(z_imu_frame);

                if matches!(baro_estimator, BaroStateEstimator::DrogueChuteDelay { .. }) {
                    *self = Self::Descent {
                        baro_estimator: baro_estimator.clone(),
                    }
                }

                result
            }
            Self::Descent { baro_estimator } => baro_estimator.update(z_imu_frame),
        }
    }
}


#[derive(Debug, Clone)]
pub struct Measurement(pub SVector<f32, { Self::SIZE }>);

impl Measurement {
    pub const SIZE: usize = 7;

    pub fn new(
        acceleration: &Vector3<f32>,
        angular_velocity: &Vector3<f32>,
        altitude_asl: f32,
    ) -> Self {
        let mut vec = SVector::<f32, 7>::zeros();
        vec.fixed_view_mut::<3, 1>(0, 0).copy_from(acceleration);
        vec.fixed_view_mut::<3, 1>(3, 0).copy_from(angular_velocity);
        vec[6] = altitude_asl;
        Self(vec)
    }

    pub fn acceleration(&self) -> Vector3<f32> {
        self.0.fixed_view::<3, 1>(0, 0).into()
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        self.0.fixed_view::<3, 1>(3, 0).into()
    }

    pub fn altitude_asl(&self) -> f32 {
        self.0[6]
    }
}

#[derive(Clone, Debug)]
pub struct FlightProfile {
    pub drogue_chute_minimum_time_us: u32,
    pub drogue_chute_minimum_altitude_agl: f32,
    pub drogue_chute_delay_us: u32,
    pub main_chute_altitude_agl: f32,
    pub main_chute_delay_us: u32,
    pub time_above_mach_08_us: u32,
}

#[cfg(test)]
mod test {
    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn state_estimator_size() {
        init_logger();
        log_info!("size: {}", size_of::<RocketStateEstimator>())
    }
}
