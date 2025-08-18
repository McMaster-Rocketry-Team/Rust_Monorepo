use std::fs::File;

use super::*;
use crate::tests::{init_logger, plot::GlobalPlot};
use csv::{Reader, Writer};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Deserialize)]
struct CsvRecord {
    time_us: f32,
    // altitude: f32,
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
    let path = "./test_data/flight_test.csv";
    let mut reader = Reader::from_reader(File::open(path).unwrap());
    reader.deserialize().map(|row| row.unwrap()).collect()
}

#[test]
fn integration_test() {
    init_logger();

    let csv_records = read_csv_records();

    let mut orientation_writer = Writer::from_path("./out_orientation.csv").unwrap();
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
        drogue_chute_minimum_time_us: 0,
        drogue_chute_minimum_altitude_agl: 10.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: 5.0,
        main_chute_delay_us: 0,
        ignition_detection_acc_threshold: 1.5 * 9.81,
    };
    let mut estimator = AscentStateEstimator::new(flight_profile);
    for csv_record in csv_records.iter() {
        let mut time_s = csv_record.time_us / 1_000_000.0;
        GlobalPlot::set_time_s(time_s);
        let reading = Measurement::new(
            &Vector3::new(
                csv_record.imu_acc_x * 9.81,
                -csv_record.imu_acc_y * 9.81,
                -csv_record.imu_acc_z * 9.81,
            ),
            &Vector3::new(
                csv_record.gyro_x.to_radians(),
                csv_record.gyro_y.to_radians(),
                csv_record.gyro_z.to_radians(),
            ),
            0.0,
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
