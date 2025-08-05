mod altitude_kf;
#[cfg(test)]
mod tests;

use firmware_common_new::{readings::BaroData, sensor_reading::SensorReading, time::TimestampType};
use nalgebra::Vector1;

use crate::{
    state_estimator::{
        DT, FlightProfile, SAMPLES_PER_S, ascent_baro::altitude_kf::AltitudeKF, welford::Welford,
    },
    utils::approximate_speed_of_sound,
};

pub enum AscentBaroStateEstimator {
    Init {
        profile: FlightProfile,
        alt_asl_welford: Welford<1>,
        n: usize,
    },
    OnPadOrAscent {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
    BaroLockOut {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
        samples_left: usize,
    },
    Coasting {
        profile: FlightProfile,
        altitude_filter: AltitudeKF,
        launch_pad_altitude_asl: f32,
    },
    DrogueChute,
}

impl AscentBaroStateEstimator {
    pub fn new(profile: FlightProfile) -> Self {
        Self::Init {
            profile,
            alt_asl_welford: Welford::<1>::new(),
            n: 0,
        }
    }

    /// return true -> deploy drogue parachute
    /// deployment delay logic is not handled here
    pub fn update(&mut self, z: &SensorReading<impl TimestampType, BaroData>) -> bool {
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
                    };
                }
            }
            Self::OnPadOrAscent {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());
                let speed_of_sound = approximate_speed_of_sound(altitude_filter.altitude());
                if altitude_filter.vertical_velocity().abs() > 0.8 * speed_of_sound {
                    let dt_us = (DT * 1_000_000f32) as usize;
                    *self = Self::BaroLockOut {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                        samples_left: profile.time_above_mach_08_us as usize / dt_us,
                    }
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
                    *self = Self::Coasting {
                        profile: profile.clone(),
                        altitude_filter: altitude_filter.clone(),
                        launch_pad_altitude_asl: *launch_pad_altitude_asl,
                    };
                }
            }
            Self::Coasting {
                profile,
                altitude_filter,
                launch_pad_altitude_asl,
            } => {
                altitude_filter.predict();
                altitude_filter.update(z.data.altitude_asl());
                if altitude_filter.altitude()
                    > profile.drogue_chute_minimum_altitude_agl + *launch_pad_altitude_asl
                    && altitude_filter.vertical_velocity() < -1.0
                {
                    *self = Self::DrogueChute
                }
            }
            Self::DrogueChute => {}
        }

        matches!(self, Self::DrogueChute)
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
            Self::Coasting {
                altitude_filter, ..
            } => Some(altitude_filter.vertical_velocity()),
            Self::DrogueChute => None,
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
            Self::Coasting {
                altitude_filter,
                launch_pad_altitude_asl,
                ..
            } => Some(altitude_filter.altitude() - launch_pad_altitude_asl),
            Self::DrogueChute => None,
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
            Self::Coasting {
                altitude_filter, ..
            } => Some(altitude_filter.altitude_variance()),
            Self::DrogueChute => None,
        }
    }
}
