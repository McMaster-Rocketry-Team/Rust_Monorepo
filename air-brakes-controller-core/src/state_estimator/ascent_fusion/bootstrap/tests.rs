use std::fs::File;

use super::*;
use crate::tests::{init_logger, plot::GlobalPlot};
use csv::Reader;
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

    let mut dead_reckoning = BootstrapStateEstimator::new();
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

        dead_reckoning.update(&reading);

        // if let RocketDeadReckoning::Stage1 {
        //     av_orientation_reckoner: reckoner,
        //     ..
        // } = &dead_reckoning
        // {
        //     let up = Vector3::new(0.0, 0.0, 1.0);
        //     let nose_direction = reckoner.orientation.inverse_transform_vector(&up);
        //     let tilt = up.angle(&nose_direction).to_degrees();

        //     GlobalPlot::add_value("Dead Reckoning Tilt", tilt);
        // }

        if let BootstrapStateEstimator::Stage2 {
            rocket_orientation_reckoner: reckoner,
            ..
        } = &dead_reckoning
        {
            GlobalPlot::add_value("Dead Reckoning Pos X", reckoner.position.x);
            GlobalPlot::add_value("Dead Reckoning Pos Y", reckoner.position.y);
            GlobalPlot::add_value("Dead Reckoning Pos Z", reckoner.position.z);
            GlobalPlot::add_value("Dead Reckoning Vel X", reckoner.velocity.x);
            GlobalPlot::add_value("Dead Reckoning Vel Y", reckoner.velocity.y);
            GlobalPlot::add_value("Dead Reckoning Vel Z", reckoner.velocity.z);

            let up = Vector3::new(0.0, 0.0, 1.0);
            let nose_direction = reckoner.orientation.inverse_transform_vector(&up);
            let tilt = up.angle(&nose_direction).to_degrees();

            GlobalPlot::add_value("Dead Reckoning Tilt", tilt);
        }
    }

    GlobalPlot::plot_all();
}
