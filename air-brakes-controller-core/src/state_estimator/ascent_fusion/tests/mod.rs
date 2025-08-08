use std::fs::File;

use super::*;
use crate::tests::{init_logger, plot::GlobalPlot};
use csv::{Reader, Writer};
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

    let constants = RocketConstants {
        initial_front_cd: [0.5, 0.5, 0.5, 0.5],
        initial_sideways_moment_co: 0.3,
        side_cd: 0.55,
        burn_out_mass: 17.625,
        moment_of_inertia: 11.11,
        front_reference_area: 0.01368,
        side_reference_area: 0.3575,
    };
    let mut estimator = AscentFusionStateEstimator::new(constants);
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

        estimator.update(0.0, &reading);

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

        GlobalPlot::add_value("Estimated altitude agl", estimator.altitude_agl());
        GlobalPlot::add_value("true altitude agl", csv_record.altitude);
        if let AscentFusionStateEstimator::Ready { estimator, .. } = &estimator{
            GlobalPlot::add_value("Estimated Cd", estimator.state.drag_coefficients()[0]);
            GlobalPlot::add_value("Estimated sideways_moment_co", estimator.state.sideways_moment_co());
        }
    }

    GlobalPlot::plot_all();
}
