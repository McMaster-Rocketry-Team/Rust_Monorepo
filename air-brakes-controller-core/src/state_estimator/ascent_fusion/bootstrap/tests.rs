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

    // Create CSV writer for orientation output
    let mut orientation_writer = Writer::from_path("./out_orientation.csv").unwrap();
    orientation_writer.write_record(&["timestamp_s", "orientation_w", "orientation_x", "orientation_y", "orientation_z"]).unwrap();

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

        // if let BootstrapStateEstimator::Stage1 {
        //     av_orientation_reckoner: reckoner,
        //     ..
        // } = &dead_reckoning
        // {
        //     let quat = reckoner.orientation.quaternion();
        //     orientation_writer.write_record(&[
        //         csv_record.timestamp_s.to_string(),
        //         quat.w.to_string(),
        //         quat.i.to_string(),
        //         quat.j.to_string(),
        //         quat.k.to_string(),
        //     ]).unwrap();
        // }

        if let BootstrapStateEstimator::Stage2 {
            av_orientation_reckoner: reckoner,
            q_av_to_rocket,
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

            // Write orientation data to CSV
            let rocket_orientation = reckoner.orientation * q_av_to_rocket;
            orientation_writer.write_record(&[
                csv_record.timestamp_s.to_string(),
                rocket_orientation.w.to_string(),
                rocket_orientation.i.to_string(),
                rocket_orientation.j.to_string(),
                rocket_orientation.k.to_string(),
            ]).unwrap();
        }
    }

    // Flush the CSV writer to ensure all data is written
    orientation_writer.flush().unwrap();

    GlobalPlot::plot_all();
}
