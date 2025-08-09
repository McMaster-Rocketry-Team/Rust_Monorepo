use nalgebra::Vector1;

use crate::{
    state_estimator2::{
        DT, FlightProfile, Measurement, SAMPLES_PER_S,
        ascent::{altitude_kf::AltitudeKF, welford::Welford},
    },
    utils::approximate_speed_of_sound,
};

#[derive(Debug, Clone)]
pub enum AltitudeEstimator {
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
    },
    Apogee {
        last_altitude_asl: f32,
        launch_pad_altitude_asl: f32,
    },
}

impl AltitudeEstimator {
    pub fn new(profile: FlightProfile) -> Self {
        Self::Init {
            profile,
            alt_asl_welford: Welford::<1>::new(),
            n: 0,
        }
    }

    pub fn update(&mut self, acc_vertical: f32, z_imu_frame: &Measurement) {
        match self {
            Self::Init {
                profile,
                alt_asl_welford,
                n,
            } => {
                alt_asl_welford.update(&Vector1::new(z_imu_frame.altitude_asl()));
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
                altitude_filter.predict(acc_vertical);
                altitude_filter.update(z_imu_frame.altitude_asl());
                let speed_of_sound = approximate_speed_of_sound(altitude_filter.altitude());
                if !*locked_out_once
                    && altitude_filter.vertical_velocity().abs() > 0.9 * speed_of_sound
                {
                    *self = Self::BaroLockOut {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    }
                } else if altitude_filter.altitude()
                    > profile.drogue_chute_minimum_altitude_agl + *launch_pad_altitude_asl
                    && altitude_filter.vertical_velocity() < -1.0
                {
                    *self = Self::Apogee {
                        last_altitude_asl: altitude_filter.altitude(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::BaroLockOut {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
            } => {
                altitude_filter.predict(acc_vertical);
                let speed_of_sound = approximate_speed_of_sound(altitude_filter.altitude());
                if altitude_filter.vertical_velocity().abs() < 0.85 * speed_of_sound {
                    *self = Self::OnPadOrAscent {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        locked_out_once: true,
                    };
                }
            }
            Self::Apogee { .. } => {}
        }
    }

    pub fn is_apogee(&self) -> bool {
        matches!(self, Self::Apogee { .. })
    }

    fn ns_to_ticks(ns: u32) -> usize {
        let dt_us = (DT * 1_000_000f32) as usize;
        ns as usize / dt_us
    }

    pub fn velocity(&self) -> f32 {
        match self {
            Self::Init { .. } => 0.0,
            Self::OnPadOrAscent {
                altitude_filter, ..
            }
            | Self::BaroLockOut {
                altitude_filter, ..
            } => altitude_filter.vertical_velocity(),
            Self::Apogee { .. } => 0.0,
        }
    }

    pub fn altitude_agl(&self) -> f32 {
        match self {
            Self::Init { .. } => 0.0,
            Self::OnPadOrAscent {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            }
            | Self::BaroLockOut {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => altitude_filter.altitude() - launch_pad_altitude_asl,
            Self::Apogee {
                last_altitude_asl,
                launch_pad_altitude_asl,
            } => last_altitude_asl - launch_pad_altitude_asl,
        }
    }

    pub fn altitude_variance(&self) -> Option<f32> {
        match self {
            Self::Init { .. } => None,
            Self::OnPadOrAscent {
                altitude_filter, ..
            }
            | Self::BaroLockOut {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::Apogee { .. } => None,
        }
    }
}
