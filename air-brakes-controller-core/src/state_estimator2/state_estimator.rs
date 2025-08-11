use firmware_common_new::{
    can_bus::messages::vl_status::FlightStage, vlp::packets::fire_pyro::PyroSelect,
};

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
        samples_left: usize,
    },
    MainChuteDeployed {
        altitude_kf: AltitudeKF,
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
                    };
                }
            }
            Self::MainChuteDelay {
                altitude_kf,
                samples_left,
            } => {
                altitude_kf.predict();
                altitude_kf.update(z_imu_frame.altitude_asl());

                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroMain);
                    *self = Self::MainChuteDeployed {
                        altitude_kf: altitude_kf.clone(),
                    };
                }
            }
            Self::MainChuteDeployed { altitude_kf } => {
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

    pub fn flight_stage(&self) -> FlightStage {
        match self {
            Self::Ascent {
                ascent_state_estimator: AscentStateEstimator::OnPad { .. },
                ..
            } => FlightStage::Armed,
            Self::Ascent {
                ascent_state_estimator,
                ..
            } => {
                if ascent_state_estimator.is_coasting() {
                    FlightStage::Coasting
                } else {
                    FlightStage::PoweredAscent
                }
            }
            Self::DrogueChuteDelay { .. } => FlightStage::Coasting,
            Self::DrogueChuteDeployed { .. } => FlightStage::DrogueDeployed,
            Self::MainChuteDelay { .. } => FlightStage::DrogueDeployed,
            Self::MainChuteDeployed { .. } => FlightStage::MainDeployed,
            Self::Landed | Self::FailedToReachMinApogee => FlightStage::Landed,
        }
    }
}

fn ns_to_ticks(ns: u32) -> usize {
    let dt_us = (DT * 1_000_000f32) as usize;
    ns as usize / dt_us
}
