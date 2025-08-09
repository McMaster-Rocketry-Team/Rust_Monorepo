use std::fs::File;

use super::*;
use crate::tests::{init_logger, plot::GlobalPlot};
use csv::{Reader, Writer};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Deserialize)]
struct CsvRecord {
    timestamp_s: f32,
    altitude: f32,
    imu_acc_x: f32,
    imu_acc_y: f32,
    imu_acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    velocity_x:f32,
    velocity_y:f32,
}

fn read_csv_records() -> Vec<CsvRecord> {
    let path = "./test_data/merged.csv";
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

    let mut estimator = AscentStateEstimator::new(FlightProfile {
        drogue_chute_minimum_time_us: 0,
        drogue_chute_minimum_altitude_agl: 2000.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: 400.0,
        main_chute_delay_us: 0,
        ignition_detection_acc_threshold: 3.0 * 9.81,
    });
    for csv_record in csv_records.iter() {
        GlobalPlot::set_time_s(csv_record.timestamp_s);
        let reading = Measurement::new(
            &Vector3::new(
                csv_record.imu_acc_x,
                csv_record.imu_acc_y,
                csv_record.imu_acc_z,
            ),
            &Vector3::new(csv_record.gyro_x, csv_record.gyro_y, csv_record.gyro_z),
            csv_record.altitude,
        );

        estimator.update(&reading);

        if let Some(rocket_orientation) = estimator.rocket_orientation() {
            orientation_writer
                .write_record(&[
                    csv_record.timestamp_s.to_string(),
                    rocket_orientation.w.to_string(),
                    rocket_orientation.i.to_string(),
                    rocket_orientation.j.to_string(),
                    rocket_orientation.k.to_string(),
                ])
                .unwrap();
        }

        if let Some((tilt, velocity)) = estimator.tilt_and_velocity() {
            GlobalPlot::add_value("Estimated tilt", tilt.to_degrees());
            GlobalPlot::add_value("Estimated horizontal velocity", velocity.x);
            let true_horizontal_velocity = Vector2::new(csv_record.velocity_x, csv_record.velocity_y).magnitude();
            GlobalPlot::add_value("True horizontal velocity", true_horizontal_velocity);
            GlobalPlot::add_value("Horizontal velocity residue", velocity.x - true_horizontal_velocity);
        }
    }

    GlobalPlot::plot_all();
}
