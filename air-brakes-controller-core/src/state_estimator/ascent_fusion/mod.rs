use crate::state_estimator::ascent_fusion::{
    bootstrap::BootstrapStateEstimator, mekf::MekfStateEstimator,
};

mod bootstrap;
mod mekf;

const SAMPLES_PER_S: usize = 500;
const DT: f32 = 1f32 / (SAMPLES_PER_S as f32);

pub enum AscentFusionStateEstimator {
    Bootstrap { estimator: BootstrapStateEstimator },
    Ready { estimator: MekfStateEstimator },
}
