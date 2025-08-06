use std::fs::File;

use csv::Reader;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use serde::Deserialize;

use super::super::state::State;
use crate::{
    state_estimator::{ascent_fusion::mekf::state_propagation::state_transition, DT}, tests::{init_logger, plot::GlobalPlot}, RocketConstants
};

#[derive(Deserialize)]
pub(super) struct CsvRecord {
    pub(super) timestamp_s: f32,
    pub(super) altitude: f32,
    pub(super) acc_x: f32,
    pub(super) acc_y: f32,
    pub(super) acc_z: f32,
    pub(super) angular_velocity_x: f32,
    pub(super) angular_velocity_y: f32,
    pub(super) angular_velocity_z: f32,
    pub(super) velocity_x: f32,
    pub(super) velocity_y: f32,
    pub(super) velocity_z: f32,
    pub(super) orientation_w: f32,
    pub(super) orientation_x: f32,
    pub(super) orientation_y: f32,
    pub(super) orientation_z: f32,
}

impl CsvRecord {
    pub(super) fn to_rocket_state(&self) -> State {
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

    pub(super) fn to_orientation(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_quaternion(Quaternion::new(
            self.orientation_w,
            self.orientation_x,
            self.orientation_y,
            self.orientation_z,
        ))
    }
}

pub(super) fn read_csv_records() -> Vec<CsvRecord> {
    let path = "./test_data/merged.csv";
    let mut reader = Reader::from_reader(File::open(path).unwrap());
    reader
        .deserialize()
        .map(|row| row.unwrap())
        .filter(|record: &CsvRecord| record.timestamp_s >= 8.5) // only include coasting data
        .collect()
}

#[test]
fn calculate_residue() {
    init_logger();

    let constants = RocketConstants {
        initial_front_cd: [0.5, 0.5, 0.5, 0.5],
        initial_sideways_moment_co: 0.3,
        side_cd: 0.55,
        burn_out_mass: 17.625,
        moment_of_inertia: 11.11,
        front_reference_area: 0.01368,
        side_reference_area: 0.3575,
    };

    let csv_records = read_csv_records();
    let current_records = csv_records.iter();
    let next_records = csv_records.iter().skip(1);

    for (csv_record, next_record) in current_records.zip(next_records) {
        GlobalPlot::set_time_s(csv_record.timestamp_s);
        let state = csv_record.to_rocket_state();
        let orientation = csv_record.to_orientation();

        let mut predicted_state = state_transition(0.0, &orientation, &state, &constants);
        let predicted_orientation = predicted_state.reset_small_angle_correction(&orientation);

        let true_state = next_record.to_rocket_state();
        let true_orientation = next_record.to_orientation();

        let d_acc_z = (next_record.acc_z - csv_record.acc_z) / DT;
        GlobalPlot::add_value("Real dAcc Z World Frame", d_acc_z);

        GlobalPlot::add_value(
            "Altitude Residue x500",
            (true_state.altitude_asl() - predicted_state.altitude_asl()) * 500.0,
        );
        GlobalPlot::add_value(
            "Acc X Residue x500",
            (true_state.acceleration().x - predicted_state.acceleration().x) * 500.0,
        );
        GlobalPlot::add_value(
            "Acc Y Residue x500",
            (true_state.acceleration().y - predicted_state.acceleration().y) * 500.0,
        );
        GlobalPlot::add_value(
            "Acc Z Residue x500",
            (true_state.acceleration().z - predicted_state.acceleration().z) * 500.0,
        );
        GlobalPlot::add_value(
            "Velocity X Residue x500",
            (true_state.velocity().x - predicted_state.velocity().x) * 500.0,
        );
        GlobalPlot::add_value(
            "Velocity Y Residue x500",
            (true_state.velocity().y - predicted_state.velocity().y) * 500.0,
        );
        GlobalPlot::add_value(
            "Velocity Z Residue x500",
            (true_state.velocity().z - predicted_state.velocity().z) * 500.0,
        );
        GlobalPlot::add_value(
            "Angular Velocity X Residue x500",
            (true_state.angular_velocity().x - predicted_state.angular_velocity().x) * 500.0,
        );
        GlobalPlot::add_value(
            "Angular Velocity Y Residue x500",
            (true_state.angular_velocity().y - predicted_state.angular_velocity().y) * 500.0,
        );
        GlobalPlot::add_value(
            "Angular Velocity Z Residue x500",
            (true_state.angular_velocity().z - predicted_state.angular_velocity().z) * 500.0,
        );

        // Calculate euler angle residues
        let predicted_euler_angle = predicted_orientation.euler_angles();
        let true_euler_angle = true_orientation.euler_angles();
        GlobalPlot::add_value(
            "Yaw Residue x500",
            (true_euler_angle.0 - predicted_euler_angle.0).to_degrees() * 500.0,
        );
        GlobalPlot::add_value(
            "Pitch Residue x500",
            (true_euler_angle.1 - predicted_euler_angle.1).to_degrees() * 500.0,
        );
        GlobalPlot::add_value(
            "Roll Residue x500",
            (true_euler_angle.2 - predicted_euler_angle.2).to_degrees() * 500.0,
        );
    }

    GlobalPlot::plot_all();
}
