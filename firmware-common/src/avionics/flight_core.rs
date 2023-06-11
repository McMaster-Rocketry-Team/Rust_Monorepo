use core::f32::consts::FRAC_PI_2;
use core::ops::Mul;
use core::ops::Neg;

use crate::common::gps_parser::GPSLocation;
use crate::common::moving_average::NoSumSMA;
use crate::common::sensor_snapshot::PartialSensorSnapshot;
use eskf::ESKF;
use ferraris_calibration::IMUReading;
use heapless::Deque;
use nalgebra::Matrix3;
use nalgebra::Point3;
use nalgebra::Scalar;
use nalgebra::Unit;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

use super::baro_reading_filter::BaroFilterOutput;
use super::baro_reading_filter::BaroReadingFilter;
use super::flight_core_event::FlightCoreEvent;
use super::flight_core_event::FlightCoreEventDispatcher;

pub struct Variances {
    pub acc: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub baro_altemeter: f32,
}

pub struct Config {
    pub drogue_chute_delay_ms: f64,
    pub main_chute_delay_ms: f64,
    pub main_chute_altitude_agl: f32,
    pub main_chute_minimum_time_ms: f64,
    pub main_chute_minimum_altitude_agl: f32,
}

pub enum FlightCoreState {
    Armed {
        // 500ms history
        snapshot_history: Deque<PartialSensorSnapshot, 100>,
        gps_location_history: Deque<GPSLocation, 3>,
        acc_y_moving_average: NoSumSMA<f32, f32, 4>,
    },
    PowerAscend {
        acc_mag_moving_average: NoSumSMA<f32, f32, 4>,
        launch_altitude: f32,
        launch_timestamp: f64,
    },
    Coast {
        launch_altitude: f32,
        launch_timestamp: f64,
    },
    DrogueChute {
        launch_altitude: f32,
        launch_timestamp: f64,
        deploy_time: f64,
    },
    MainChute {
        launch_altitude: f32,
        deploy_time: Option<f64>,
    },
    MainChuteDescend {},
    Landed {},
}

impl FlightCoreState {
    pub fn new() -> Self {
        Self::Armed {
            snapshot_history: Deque::new(),
            gps_location_history: Deque::new(),
            acc_y_moving_average: NoSumSMA::new(0.0),
        }
    }

    pub fn is_in_air(&self) -> bool {
        match self {
            Self::Armed { .. } => false,
            _ => true,
        }
    }
}

// TODO throw critical error when too many eskf updates fail
// Designed to run at 200hz
pub struct FlightCore<D: FlightCoreEventDispatcher> {
    event_dispatcher: D,
    config: Config,
    state: FlightCoreState,
    mounting_angle_compensation_quat: UnitQuaternion<f32>,
    last_snapshot_timestamp: Option<f64>,
    baro_altimeter_offset: Option<f32>,
    eskf: ESKF,
    variances: Variances,
    baro_filter: BaroReadingFilter,
    critical_error: bool,
}

impl<D: FlightCoreEventDispatcher> FlightCore<D> {
    pub fn new(
        config: Config,
        event_dispatcher: D,
        rocket_upright_imu_reading: IMUReading,
        variances: Variances,
    ) -> Self {
        let upright_gravity_vector = Vector3::from(rocket_upright_imu_reading.acc);
        let sky_vector = -upright_gravity_vector.normalize();
        let plus_y_vector = Vector3::<f32>::new(0.0, 1.0, 0.0);
        Self {
            event_dispatcher,
            config,
            baro_altimeter_offset: None,
            state: FlightCoreState::new(),
            // panics when sky_vector and plus_y_vector are pointing in the opposite direction,
            // which means the avionics is mounted exactly upside down, will likely not happen irl
            mounting_angle_compensation_quat: UnitQuaternion::rotation_between(
                &sky_vector,
                &plus_y_vector,
            )
            .unwrap(),
            last_snapshot_timestamp: None,
            eskf: eskf::Builder::new()
                .acceleration_variance(variances.acc.magnitude())
                .rotation_variance(variances.gyro.magnitude())
                .initial_covariance(1e-1)
                .build(),
            variances,
            baro_filter: BaroReadingFilter::new(),
            critical_error: false,
        }
    }

    pub fn tick(&mut self, mut snapshot: PartialSensorSnapshot) {
        if self.critical_error {
            return;
        }
        if self.last_snapshot_timestamp.is_none() {
            self.last_snapshot_timestamp = Some(snapshot.timestamp);
            return;
        }

        let dt = snapshot.imu_reading.timestamp - self.last_snapshot_timestamp.unwrap();

        // apply mounting angle compensation
        let acc = self
            .mounting_angle_compensation_quat
            .mul(&Vector3::from(snapshot.imu_reading.acc));
        snapshot.imu_reading.acc = acc.clone().into();

        let gyro = self
            .mounting_angle_compensation_quat
            .mul(&Vector3::from(snapshot.imu_reading.gyro));
        snapshot.imu_reading.gyro = gyro.clone().into();

        if self.state.is_in_air() {
            self.eskf.predict(
                acc.clone().y_up_to_z_up(),
                gyro.clone().y_up_to_z_up(),
                (dt / 1000.0) as f32,
            );

            if let Some(baro_reading) = &snapshot.baro_reading &&
            let BaroFilterOutput {
                should_ignore: false,
                baro_reading,
            } = self.baro_filter.feed(baro_reading)
            {
                let mut altitude = baro_reading.altitude();
                if let Some(baro_altimeter_offset) = self.baro_altimeter_offset {
                    altitude += baro_altimeter_offset;
                }
                self.eskf.observe_height(altitude, self.variances.baro_altemeter).ok();
            }

            if let Some(GPSLocation {
                altitude: Some(altitude),
                vdop: Some(vdop),
                ..
            }) = &snapshot.gps_location
            {
                // TODO use gps location infomation
                self.eskf.observe_height(*altitude, vdop * vdop).ok();
            }
        }

        match &mut self.state {
            FlightCoreState::Armed {
                snapshot_history,
                gps_location_history,
                acc_y_moving_average,
            } => {
                // update state
                acc_y_moving_average.add_sample(snapshot.imu_reading.acc[1]);

                if snapshot_history.is_full() {
                    snapshot_history.pop_front();
                }
                snapshot_history.push_back(snapshot.clone()).unwrap();

                if let Some(gps_location) = &snapshot.gps_location {
                    if gps_location_history.is_full() {
                        gps_location_history.pop_front();
                    }
                    gps_location_history
                        .push_back(gps_location.clone())
                        .unwrap();
                }

                // launch detection
                if snapshot_history.is_full() && acc_y_moving_average.get_average() < -50.0 {
                    // backtrack 500ms to calculate launch angle
                    let snapshot_before_launch = snapshot_history.front().unwrap();

                    let launch_vector =
                        -Vector3::from(snapshot_before_launch.imu_reading.acc).normalize();
                    let sky_vector = Vector3::<f32>::new(0.0, 1.0, 0.0);
                    // panics when sky_vector and plus_y_vector are pointing in the opposite direction,
                    // which means the rocket is nose down, if thats the case we got bigger problems
                    let orientation =
                        UnitQuaternion::rotation_between(&sky_vector, &launch_vector).unwrap();
                    let observe_result = self
                        .eskf
                        .observe_orientation(orientation.y_up_to_z_up(), Matrix3::zeros());
                    if observe_result.is_err() {
                        self.critical_error = true;
                        self.event_dispatcher
                            .dispatch(FlightCoreEvent::CriticalError);
                        return;
                    }

                    let mut gps_altitude_sum: f32 = 0.0;
                    let mut gps_altitude_count: u32 = 0;
                    for gps_location in gps_location_history.iter() {
                        if gps_location.timestamp < snapshot_before_launch.timestamp &&
                        gps_location.timestamp > snapshot_before_launch.timestamp - 5000.0 &&
                        let Some(speed) = gps_location.speed_over_ground && speed < 0.2 &&
                        let Some(gps_altitude) = gps_location.altitude
                        {
                            gps_altitude_sum += gps_altitude;
                            gps_altitude_count += 1;
                        }
                    }

                    let baro_before_launch = snapshot_history
                        .iter()
                        .find(|s| s.baro_reading.is_some())
                        .map(|s| s.baro_reading.as_ref().unwrap().altitude());

                    let launch_altitude = if gps_altitude_count > 0 {
                        let launch_altitude = gps_altitude_sum / gps_altitude_count as f32;
                        if let Some(baro_before_launch) = baro_before_launch {
                            self.baro_altimeter_offset = Some(launch_altitude - baro_before_launch);
                        }
                        launch_altitude
                    } else if let Some(baro_altitude) = baro_before_launch {
                        baro_altitude
                    } else {
                        0.0
                    };
                    let observe_result = self.eskf.observe_height(launch_altitude, 0.0);
                    if observe_result.is_err() {
                        self.critical_error = true;
                        self.event_dispatcher
                            .dispatch(FlightCoreEvent::CriticalError);
                        return;
                    }

                    for (prev_snapshot, snapshot) in
                        snapshot_history.iter().zip(snapshot_history.iter().skip(1))
                    {
                        self.eskf.predict(
                            Vector3::from(snapshot.imu_reading.acc).y_up_to_z_up(),
                            Vector3::from(snapshot.imu_reading.gyro).y_up_to_z_up(),
                            ((snapshot.timestamp - prev_snapshot.timestamp) / 1000.0) as f32,
                        );
                    }

                    self.event_dispatcher.dispatch(FlightCoreEvent::Ignition);
                    self.state = FlightCoreState::PowerAscend {
                        launch_timestamp: snapshot.timestamp,
                        launch_altitude,
                        acc_mag_moving_average: NoSumSMA::new(0.0),
                    };
                }
            }
            FlightCoreState::PowerAscend {
                acc_mag_moving_average,
                launch_timestamp,
                launch_altitude,
            } => {
                acc_mag_moving_average
                    .add_sample(Vector3::from(snapshot.imu_reading.acc).magnitude());

                // coast detection
                if acc_mag_moving_average.is_full() && acc_mag_moving_average.get_average() < 10.0 {
                    self.state = FlightCoreState::Coast {
                        launch_timestamp: *launch_timestamp,
                        launch_altitude: *launch_altitude,
                    };
                }
            }
            FlightCoreState::Coast {
                launch_timestamp,
                launch_altitude,
            } => {
                // apogee detection
                if self.eskf.velocity.z_up_to_y_up().y <= 0.0 {
                    self.event_dispatcher.dispatch(FlightCoreEvent::Apogee);
                    self.state = FlightCoreState::DrogueChute {
                        deploy_time: snapshot.timestamp + self.config.drogue_chute_delay_ms,
                        launch_altitude: *launch_altitude,
                        launch_timestamp: *launch_timestamp,
                    };
                }
            }
            FlightCoreState::DrogueChute {
                deploy_time,
                launch_altitude,
                launch_timestamp,
            } => {
                let altitude_agl = self.eskf.position.z_up_to_y_up().y - *launch_altitude;
                if altitude_agl < self.config.main_chute_minimum_altitude_agl {
                    self.event_dispatcher.dispatch(FlightCoreEvent::DidNotReachMinApogee);
                    self.state = FlightCoreState::Landed {};
                } else {
                    if snapshot.timestamp >= *deploy_time
                        && snapshot.timestamp - *launch_timestamp
                            >= self.config.main_chute_minimum_time_ms
                    {
                        self.event_dispatcher
                            .dispatch(FlightCoreEvent::DeployDrogue);
                        self.state = FlightCoreState::MainChute {
                            deploy_time: None,
                            launch_altitude: *launch_altitude,
                        };
                    }
                }
            }
            FlightCoreState::MainChute {
                deploy_time: None,
                launch_altitude,
            } => {
                let altitude_agl = self.eskf.position.z_up_to_y_up().y - *launch_altitude;
                if altitude_agl <= self.config.main_chute_altitude_agl {
                    self.state = FlightCoreState::MainChute {
                        deploy_time: Some(snapshot.timestamp + self.config.main_chute_delay_ms),
                        launch_altitude: *launch_altitude,
                    };
                }
            }
            FlightCoreState::MainChute {
                deploy_time: Some(deploy_time),
                launch_altitude: _,
            } => {
                if snapshot.timestamp >= *deploy_time {
                    self.event_dispatcher.dispatch(FlightCoreEvent::DeployMain);
                    self.state = FlightCoreState::MainChuteDescend {};
                }
            }
            FlightCoreState::MainChuteDescend {} => {
                // landing detection
                if self.eskf.velocity.magnitude() < 0.5 {
                    self.event_dispatcher.dispatch(FlightCoreEvent::Landed);
                    self.state = FlightCoreState::Landed {};
                }
            }
            FlightCoreState::Landed {} => {}
        }

        self.last_snapshot_timestamp = Some(snapshot.timestamp);
    }
}

// The coordinate system used by the flight core and visualizer is Y up, Z forward, X right:
//     Y
//     |
//     |
//     |_______ X
//    /
//   /
//  Z

// The coordinate system used by eskf is Z up, Y backward, X right:
//     Z    Y
//     |  /
//     | /
//     |_______ X

trait CoordinateSystemConvertable<T> {
    fn y_up_to_z_up(self) -> T;
    fn z_up_to_y_up(self) -> T;
}

impl<T: Scalar + Neg<Output = T> + Copy> CoordinateSystemConvertable<Vector3<T>> for Vector3<T> {
    fn y_up_to_z_up(self) -> Vector3<T> {
        Vector3::new(self.x, -self.z, self.y)
    }

    fn z_up_to_y_up(self) -> Vector3<T> {
        Vector3::new(self.x, self.z, -self.y)
    }
}

impl<T: Scalar + Neg<Output = T> + Copy> CoordinateSystemConvertable<Point3<T>> for Point3<T> {
    fn y_up_to_z_up(self) -> Point3<T> {
        Point3::new(self.x, -self.z, self.y)
    }

    fn z_up_to_y_up(self) -> Point3<T> {
        Point3::new(self.x, self.z, -self.y)
    }
}

impl CoordinateSystemConvertable<UnitQuaternion<f32>> for UnitQuaternion<f32> {
    fn y_up_to_z_up(self) -> UnitQuaternion<f32> {
        let rot_quat =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::x()), -FRAC_PI_2);
        rot_quat * self
    }

    fn z_up_to_y_up(self) -> UnitQuaternion<f32> {
        let rot_quat =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::x()), -FRAC_PI_2);
        rot_quat * self
    }
}