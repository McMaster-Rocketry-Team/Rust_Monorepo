use nalgebra::{SVector, Vector3};

use crate::state_estimator::{
    ascent_baro::AscentBaroStateEstimator, ascent_fusion::AscentFusionStateEstimator,
    descent::DescentStateEstimator,
};

mod ascent_baro;
mod ascent_fusion;
mod descent;

pub enum RocketStateEstimator {
    Ascent {
        baro_estimator: AscentBaroStateEstimator,
        fusion_estimator: AscentFusionStateEstimator,
    },
    Descent {
        estimator: DescentStateEstimator,
    },
}

/// Values are in imu frame
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
