use std::fs::File;

use csv::Reader;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use plotters::prelude::*;
use serde::Deserialize;

use crate::{
    mekf::{state_propagation::calculate_state_derivative, State, RocketConstants},
    tests::init_logger,
};

#[derive(Deserialize)]
struct CsvRecord {
    timestamp_s: f32,
    altitude: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    angular_velocity_x: f32,
    angular_velocity_y: f32,
    angular_velocity_z: f32,
    velocity_x: f32,
    velocity_y: f32,
    velocity_z: f32,
    orientation_w: f32,
    orientation_x: f32,
    orientation_y: f32,
    orientation_z: f32,
}

impl CsvRecord {
    fn to_rocket_state(&self) -> State {
        State::new(
            &Vector3::zeros(), // small angle correction starts at zero
            &Vector3::new(self.acc_x, self.acc_y, self.acc_z),
            &Vector3::new(self.velocity_x, self.velocity_y, self.velocity_z),
            &Vector3::new(
                self.angular_velocity_x,
                self.angular_velocity_y,
                self.angular_velocity_z,
            ),
            self.altitude,
            0.3, // sideways moment coefficient (using default value)
            &Vector4::new(0.5, 0.5, 0.5, 0.5), // drag coefficients (using default values)
        )
    }

    fn to_orientation(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_quaternion(Quaternion::new(
            self.orientation_w,
            self.orientation_x,
            self.orientation_y,
            self.orientation_z,
        ))
    }
}

fn read_csv_records() -> Vec<CsvRecord> {
    let path = "./test_data/merged_world_frame.csv";
    let mut reader = Reader::from_reader(File::open(path).unwrap());
    reader
        .deserialize()
        .map(|row| row.unwrap())
        .filter(|record: &CsvRecord| record.timestamp_s >= 6.5) // only include coasting data
        .collect()
}

fn min_max_range(values: &[f32]) -> std::ops::Range<f32> {
    let min = values.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max = values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    min..max
}

fn plot_altitude_residues(
    data: &Vec<(f32, f32)>,
    data_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let file_path = format!(
        "plots_out/{}_vs_time.png",
        data_name.to_lowercase().replace(" ", "_")
    );
    let root = BitMapBackend::new(&file_path, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // Extract time and altitude ranges more cleanly
    let time_range = min_max_range(&data.iter().map(|(t, _)| *t).collect::<Vec<f32>>());
    let value_range = min_max_range(&data.iter().map(|(_, a)| *a).collect::<Vec<f32>>());
    log_info!("value range for {}: {:?}", data_name, value_range);

    let mut chart = ChartBuilder::on(&root)
        .caption(format!("{data_name} vs Time"), ("sans-serif", 40))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range, value_range)?;

    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc(data_name)
        .draw()?;

    chart
        .draw_series(LineSeries::new(data.iter().cloned(), &RED))?
        .label(data_name)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;
    log_info!("Plot saved as {}", &file_path);
    Ok(())
}

#[test]
fn calculate_residue() {
    init_logger();

    let constants = RocketConstants {
        side_cd: 0.5,
        burn_out_mass: 17.625,
        moment_of_inertia: 11.11,
        front_reference_area: 0.01368,
        side_reference_area: 0.3575,
    };
    let delta_time = 1.0f32 / 200.0;

    let csv_records = read_csv_records();
    let current_records = csv_records.iter();
    let next_records = csv_records.iter().skip(1);

    let mut residues = Vec::<CsvRecord>::new();
    let mut euler_angle_residues = Vec::<(f32, f32, f32, f32)>::new();

    for (csv_record, next_record) in current_records.zip(next_records) {
        let state = csv_record.to_rocket_state();
        let orientation = csv_record.to_orientation();
        let derivative = calculate_state_derivative(0.0, &orientation, &state, &constants);

        let mut predicted_state = state.add_derivative(&derivative, delta_time);
        let predicted_orientation = predicted_state.reset_small_angle_correction(&orientation);

        let true_state = next_record.to_rocket_state();
        let true_orientation = next_record.to_orientation();

        residues.push(CsvRecord {
            timestamp_s: csv_record.timestamp_s,
            altitude: true_state.altitude_asl() - predicted_state.altitude_asl(),
            acc_x: true_state.acceleration().x - predicted_state.acceleration().x,
            acc_y: true_state.acceleration().y - predicted_state.acceleration().y,
            acc_z: true_state.acceleration().z - predicted_state.acceleration().z,
            angular_velocity_x: true_state.angular_velocity().x
                - predicted_state.angular_velocity().x,
            angular_velocity_y: true_state.angular_velocity().y
                - predicted_state.angular_velocity().y,
            angular_velocity_z: true_state.angular_velocity().z
                - predicted_state.angular_velocity().z,
            velocity_x: true_state.velocity().x - predicted_state.velocity().x,
            velocity_y: true_state.velocity().y - predicted_state.velocity().y,
            velocity_z: true_state.velocity().z - predicted_state.velocity().z,
            orientation_w: true_orientation.w - predicted_orientation.w,
            orientation_x: true_orientation.i - predicted_orientation.i,
            orientation_y: true_orientation.j - predicted_orientation.j,
            orientation_z: true_orientation.k - predicted_orientation.k,
        });

        let predicted_euler_angle = predicted_orientation.euler_angles();
        let true_euler_angle = true_orientation.euler_angles();
        euler_angle_residues.push((
            csv_record.timestamp_s,
            true_euler_angle.0 - predicted_euler_angle.0,
            true_euler_angle.1 - predicted_euler_angle.1,
            true_euler_angle.2 - predicted_euler_angle.2,
        ));
    }

    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.altitude * 200.0))
            .collect(),
        "Altitude Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.acc_x * 200.0))
            .collect(),
        "Acc X Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.acc_y * 200.0))
            .collect(),
        "Acc Y Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.acc_z * 200.0))
            .collect(),
        "Acc Z Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.velocity_x * 200.0))
            .collect(),
        "Velocity X Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.velocity_y * 200.0))
            .collect(),
        "Velocity Y Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.velocity_z * 200.0))
            .collect(),
        "Velocity Z Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.angular_velocity_x * 200.0))
            .collect(),
        "Angular Velocity X Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.angular_velocity_y * 200.0))
            .collect(),
        "Angular Velocity Y Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &residues
            .iter()
            .map(|r| (r.timestamp_s, r.angular_velocity_z * 200.0))
            .collect(),
        "Angular Velocity Z Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &euler_angle_residues
            .iter()
            .map(|r| (r.0, r.1 * 200.0))
            .collect(),
        "Yaw Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &euler_angle_residues
            .iter()
            .map(|r| (r.0, r.2 * 200.0))
            .collect(),
        "Pitch Residue x200",
    )
    .unwrap();
    plot_altitude_residues(
        &euler_angle_residues
            .iter()
            .map(|r| (r.0, r.3 * 200.0))
            .collect(),
        "Roll Residue x200",
    )
    .unwrap();
}
