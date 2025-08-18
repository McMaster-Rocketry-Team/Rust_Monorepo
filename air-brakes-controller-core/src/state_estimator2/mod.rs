use nalgebra::{SVector, Vector3};

mod ascent;
mod descent;
mod state_estimator;

pub use state_estimator::{RocketStateEstimator, RocketState};

const SAMPLES_PER_S: usize = 416;
const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct FlightProfile {
    pub ignition_detection_acc_threshold: f32,
    pub drogue_chute_minimum_time_us: u32,
    pub drogue_chute_minimum_altitude_agl: f32,
    pub drogue_chute_delay_us: u32,
    pub main_chute_altitude_agl: f32,
    pub main_chute_delay_us: u32,
}
