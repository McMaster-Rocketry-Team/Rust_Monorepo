use std::fs::File;

use super::*;
use crate::{
    AirBrakesMPC, RocketParameters,
    controller::{State, rocket_dynamics::simulate_apogee_rk2},
    tests::{init_logger, plot::GlobalPlot},
};
use csv::{Reader, Writer};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Deserialize)]
struct CsvRecord {
    time_us: f32,
    altitude: f32,
    imu_acc_x: f32,
    imu_acc_y: f32,
    imu_acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    // velocity_x: f32,
    // velocity_y: f32,
}

fn read_csv_records() -> Vec<CsvRecord> {
    let path = "./test_data/lc_25.csv";
    let mut reader = Reader::from_reader(File::open(path).unwrap());
    reader.deserialize().map(|row| row.unwrap()).collect()
}

#[test]
fn integration_test() {
    init_logger();

    let csv_records = read_csv_records();

    let mut orientation_writer = Writer::from_path("./out_orientation.csv").unwrap();
    // can be visualized with https://quaternion-visualizer.vercel.app/
    orientation_writer
        .write_record(&[
            "timestamp_s",
            "orientation_w",
            "orientation_x",
            "orientation_y",
            "orientation_z",
        ])
        .unwrap();

    let flight_profile = FlightProfile {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        drogue_chute_minimum_time_us: 20_000_000,
        drogue_chute_minimum_altitude_agl: 2000.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: 457.2,
        main_chute_delay_us: 0,
    };
    let mut estimator = AscentStateEstimator::new(flight_profile);
    let mut airbrakes_mpc = AirBrakesMPC::new(
        RocketParameters {
            burnout_mass: 17.607,
            cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
            reference_area: 0.008982476,
        },
        296.0 + 5259.0,
    );

    for csv_record in csv_records.iter() {
        let time_s = csv_record.time_us / 1_000_000.0;
        GlobalPlot::set_time_s(time_s);
        let reading = Measurement::new(
            &Vector3::new(
                csv_record.imu_acc_x,
                -csv_record.imu_acc_y,
                -csv_record.imu_acc_z,
            ),
            &Vector3::new(
                csv_record.gyro_x.to_radians(),
                csv_record.gyro_y.to_radians(),
                csv_record.gyro_z.to_radians(),
            ),
            csv_record.altitude,
        );

        estimator.update(&reading);

        if let Some(rocket_orientation) = estimator.rocket_orientation() {
            orientation_writer
                .write_record(&[
                    time_s.to_string(),
                    rocket_orientation.w.to_string(),
                    rocket_orientation.i.to_string(),
                    rocket_orientation.j.to_string(),
                    rocket_orientation.k.to_string(),
                ])
                .unwrap();
        }

        if let Some(velocity) = estimator.velocity() {
            let tilt = velocity.angle(&Vector2::new(0.0, 1.0));
            GlobalPlot::add_value("Estimated tilt", tilt.to_degrees());
            GlobalPlot::add_value("Estimated vertical velocity", velocity.y);
            GlobalPlot::add_value("Estimated horizontal velocity", velocity.x);
            GlobalPlot::add_value("Estimated altitude agl", estimator.altitude_agl());

            const ROCKET_PARAMETERS: RocketParameters = RocketParameters {
                burnout_mass: 17.607,
                cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
                reference_area: 0.008982476,
            };
            let predicted_apogee = simulate_apogee_rk2(
                0.0,
                &State {
                    altitude_asl: estimator.altitude_asl().unwrap_or(0.0),
                    velocity,
                },
                &ROCKET_PARAMETERS,
            );
            GlobalPlot::add_value("Predicted Apogee", predicted_apogee);

            let airbrake_extension_percentage =
                airbrakes_mpc.update(estimator.altitude_asl().unwrap_or(0.0), velocity);
            GlobalPlot::add_value(
                "Airbrake extension percentage",
                airbrake_extension_percentage,
            );

            // GlobalPlot::add_value("Estimated altitude", velocity.x);
            // let true_horizontal_velocity =
            //     Vector2::new(csv_record.velocity_x, csv_record.velocity_y).magnitude();
            // GlobalPlot::add_value("True horizontal velocity", true_horizontal_velocity);
            // GlobalPlot::add_value(
            //     "Horizontal velocity residue",
            //     velocity.x - true_horizontal_velocity,
            // );
        }
    }

    GlobalPlot::plot_all();
}
