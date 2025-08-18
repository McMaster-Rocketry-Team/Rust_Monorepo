use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use nalgebra::Vector2;

use crate::state_estimator2::{
    DT, FlightProfile, Measurement, ascent::AscentStateEstimator, descent::altitude_kf::AltitudeKF,
};

pub enum RocketStateEstimator {
    Ascent {
        profile: FlightProfile,
        ascent_state_estimator: AscentStateEstimator,
    },
    DrogueChuteDelay {
        profile: FlightProfile,
        altitude_kf: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    DrogueChuteDeployed {
        profile: FlightProfile,
        altitude_kf: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
    MainChuteDelay {
        altitude_kf: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    MainChuteDeployed {
        altitude_kf: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
    Landed,
    FailedToReachMinApogee,
}

impl RocketStateEstimator {
    pub fn new(profile: FlightProfile) -> Self {
        Self::Ascent {
            profile: profile.clone(),
            ascent_state_estimator: AscentStateEstimator::new(profile),
        }
    }

    pub fn update(&mut self, z_imu_frame: &Measurement) -> Option<PyroSelect> {
        let mut deploy_pyro = None;

        match self {
            Self::Ascent {
                profile,
                ascent_state_estimator,
            } => {
                ascent_state_estimator.update(z_imu_frame);

                if let AscentStateEstimator::Apogee {
                    altitude_asl,
                    alt_variance,
                    launch_pad_altitude_asl,
                } = &ascent_state_estimator
                {
                    let altitude_agl = altitude_asl - launch_pad_altitude_asl;
                    if altitude_agl < profile.drogue_chute_minimum_altitude_agl {
                        log_info!(
                            "altitude asl: {}, pad asl: {}",
                            altitude_asl,
                            launch_pad_altitude_asl
                        );
                        log_info!(
                            "failed to reach min apogee: min={}, current={}",
                            profile.drogue_chute_minimum_altitude_agl,
                            altitude_agl
                        );
                        *self = Self::FailedToReachMinApogee;
                    } else {
                        *self = Self::DrogueChuteDelay {
                            profile: profile.clone(),
                            altitude_kf: AltitudeKF::new(*altitude_asl, *alt_variance),
                            launch_pad_altitude_asl: *launch_pad_altitude_asl,
                            samples_left: ns_to_ticks(profile.drogue_chute_delay_us),
                        }
                    }
                }
            }
            Self::DrogueChuteDelay {
                profile,
                altitude_kf,
                launch_pad_altitude_asl,
                samples_left,
            } => {
                altitude_kf.predict();
                altitude_kf.update(z_imu_frame.altitude_asl());

                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroDrogue);
                    *self = Self::DrogueChuteDeployed {
                        profile: profile.clone(),
                        altitude_kf: altitude_kf.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::DrogueChuteDeployed {
                profile,
                altitude_kf,
                launch_pad_altitude_asl,
            } => {
                altitude_kf.predict();
                altitude_kf.update(z_imu_frame.altitude_asl());

                if altitude_kf.altitude()
                    < profile.main_chute_altitude_agl + *launch_pad_altitude_asl
                {
                    *self = Self::MainChuteDelay {
                        altitude_kf: altitude_kf.clone(),
                        samples_left: ns_to_ticks(profile.main_chute_delay_us),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::MainChuteDelay {
                altitude_kf,
                samples_left,
                launch_pad_altitude_asl,
            } => {
                altitude_kf.predict();
                altitude_kf.update(z_imu_frame.altitude_asl());

                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroMain);
                    *self = Self::MainChuteDeployed {
                        altitude_kf: altitude_kf.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::MainChuteDeployed { altitude_kf, .. } => {
                altitude_kf.predict();
                altitude_kf.update(z_imu_frame.altitude_asl());

                if altitude_kf.vertical_velocity().abs() < 1.0 {
                    *self = Self::Landed;
                }
            }
            Self::Landed | Self::FailedToReachMinApogee => {}
        }

        deploy_pyro
    }

    pub fn state(&self) -> RocketState {
        match self {
            Self::Ascent {
                ascent_state_estimator: AscentStateEstimator::OnPad { .. },
                ..
            } => RocketState::OnPad,
            Self::Ascent {
                ascent_state_estimator: estimator,
                ..
            } => {
                if let Some(velocity) = estimator.velocity()
                    && let Some(altitude_asl) = estimator.altitude_asl()
                    && let Some(launch_pad_altitude_asl) = estimator.launch_pad_altitude_asl()
                {
                    if estimator.is_coasting() {
                        RocketState::PoweredAscent {
                            velocity: velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        }
                    } else {
                        RocketState::Coasting {
                            velocity: velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        }
                    }
                } else {
                    RocketState::OnPad
                }
            }
            Self::DrogueChuteDelay {
                altitude_kf,
                launch_pad_altitude_asl,
                ..
            } => RocketState::DrogueChute {
                deployed: false,
                vertical_velocity: altitude_kf.vertical_velocity(),
                altitude_asl: altitude_kf.altitude(),
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Self::DrogueChuteDeployed {
                altitude_kf,
                launch_pad_altitude_asl,
                ..
            } => RocketState::DrogueChute {
                deployed: true,
                vertical_velocity: altitude_kf.vertical_velocity(),
                altitude_asl: altitude_kf.altitude(),
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Self::MainChuteDelay {
                altitude_kf,
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: false,
                vertical_velocity: altitude_kf.vertical_velocity(),
                altitude_asl: altitude_kf.altitude(),
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Self::MainChuteDeployed {
                altitude_kf,
                launch_pad_altitude_asl,
                ..
            } => RocketState::MainChute {
                deployed: true,
                vertical_velocity: altitude_kf.vertical_velocity(),
                altitude_asl: altitude_kf.altitude(),
                launch_pad_altitude_asl: *launch_pad_altitude_asl,
            },
            Self::Landed => RocketState::Landed,
            Self::FailedToReachMinApogee => RocketState::FailedToReachMinApogee,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub enum RocketState {
    OnPad,
    PoweredAscent {
        velocity: Vector2<f32>,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    Coasting {
        velocity: Vector2<f32>,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    DrogueChute {
        deployed: bool,
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    MainChute {
        deployed: bool,
        vertical_velocity: f32,
        altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
    Landed,
    FailedToReachMinApogee,
}

fn ns_to_ticks(ns: u32) -> usize {
    let dt_us = (DT * 1_000_000f32) as usize;
    ns as usize / dt_us
}
