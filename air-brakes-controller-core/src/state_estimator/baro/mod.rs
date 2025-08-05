mod altitude_kf;
#[cfg(test)]
mod tests;

use firmware_common_new::{
    readings::BaroData, sensor_reading::SensorReading, time::TimestampType,
    vlp::packets::fire_pyro::PyroSelect,
};
use nalgebra::Vector1;

use crate::{
    state_estimator::{
        DT, FlightProfile, SAMPLES_PER_S, baro::altitude_kf::AltitudeKF, welford::Welford,
    },
    utils::approximate_speed_of_sound,
};

pub enum BaroStateEstimator {
    Init {
        profile: FlightProfile,
        alt_asl_welford: Welford<1>,
        n: usize,
    },
    OnPadOrAscent {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
        locked_out_once: bool,
    },
    BaroLockOut {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    DrogueChuteDelay {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    DrogueChuteDeployed {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
    MainChuteDelay {
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    MainChuteDeployed {
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
}

impl BaroStateEstimator {
    pub fn new(profile: FlightProfile) -> Self {
        Self::Init {
            profile,
            alt_asl_welford: Welford::<1>::new(),
            n: 0,
        }
    }

    pub fn update(
        &mut self,
        z: &SensorReading<impl TimestampType, BaroData>,
    ) -> Option<PyroSelect> {
        let mut deploy_pyro = None;
        match self {
            Self::Init {
                profile,
                alt_asl_welford,
                n,
            } => {
                alt_asl_welford.update(&Vector1::new(z.data.altitude_asl()));
                *n += 1;

                if *n == SAMPLES_PER_S {
                    let launch_pad_altitude_asl = alt_asl_welford.mean()[0];
                    let variance = alt_asl_welford.variance().unwrap()[0];
                    *self = Self::OnPadOrAscent {
                        profile: profile.clone(),
                        altitude_filter: AltitudeKF::new(launch_pad_altitude_asl, variance),
                        launch_pad_altitude_asl,
                        locked_out_once: false,
                    };
                }
            }
            Self::OnPadOrAscent {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
                locked_out_once,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());
                let speed_of_sound = approximate_speed_of_sound(altitude_filter.altitude());
                if !*locked_out_once
                    && altitude_filter.vertical_velocity().abs() > 0.8 * speed_of_sound
                {
                    *self = Self::BaroLockOut {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: Self::ns_to_ticks(profile.time_above_mach_08_us),
                    }
                } else if altitude_filter.altitude()
                    > profile.drogue_chute_minimum_altitude_agl + *launch_pad_altitude_asl
                    && altitude_filter.vertical_velocity() < -1.0
                {
                    *self = Self::DrogueChuteDelay {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: Self::ns_to_ticks(profile.drogue_chute_delay_us),
                    };
                }
            }
            Self::BaroLockOut {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
                samples_left,
            } => {
                altitude_filter.predict();
                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    *self = Self::OnPadOrAscent {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        locked_out_once: true,
                    };
                }
            }
            Self::DrogueChuteDelay {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
                samples_left,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());

                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroDrogue);
                    *self = Self::DrogueChuteDeployed {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::DrogueChuteDeployed {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());

                if altitude_filter.altitude()
                    < profile.main_chute_altitude_agl + *launch_pad_altitude_asl
                {
                    *self = Self::MainChuteDelay {
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: Self::ns_to_ticks(profile.main_chute_delay_us),
                    };
                }
            }
            Self::MainChuteDelay {
                altitude_filter,
                launch_pad_altitude_asl,
                samples_left,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());

                *samples_left = samples_left.saturating_sub(1);

                if *samples_left == 0 {
                    deploy_pyro = Some(PyroSelect::PyroMain);
                    *self = Self::MainChuteDeployed {
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::MainChuteDeployed {
                altitude_filter, ..
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());
            }
        }

        deploy_pyro
    }

    fn ns_to_ticks(ns: u32) -> usize {
        let dt_us = (DT * 1_000_000f32) as usize;
        ns as usize / dt_us
    }

    pub fn velocity(&self) -> Option<f32> {
        match self {
            Self::Init { .. } => Some(0.0),
            Self::OnPadOrAscent {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::BaroLockOut {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::DrogueChuteDelay {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::DrogueChuteDeployed {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::MainChuteDelay {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::MainChuteDeployed {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
        }
    }

    pub fn altitude_agl(&self) -> Option<f32> {
        match self {
            Self::Init { .. } => Some(0.0),
            Self::OnPadOrAscent {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::BaroLockOut {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::DrogueChuteDelay {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::DrogueChuteDeployed {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::MainChuteDelay {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::MainChuteDeployed {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
        }
    }

    pub fn altitude_variance(&self) -> Option<f32> {
        match self {
            Self::Init { .. } => None,
            Self::OnPadOrAscent {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::BaroLockOut {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::DrogueChuteDelay {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::DrogueChuteDeployed {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::MainChuteDelay {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::MainChuteDeployed {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
        }
    }
}
