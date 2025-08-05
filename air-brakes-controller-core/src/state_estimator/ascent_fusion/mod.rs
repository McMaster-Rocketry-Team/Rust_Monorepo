use crate::state_estimator::ascent_fusion::{
    bootstrap::BootstrapStateEstimator, mekf::MekfStateEstimator,
};

mod bootstrap;
mod mekf;

pub enum AscentFusionStateEstimator {
    Bootstrap { estimator: BootstrapStateEstimator },
    Ready { estimator: MekfStateEstimator },
}
