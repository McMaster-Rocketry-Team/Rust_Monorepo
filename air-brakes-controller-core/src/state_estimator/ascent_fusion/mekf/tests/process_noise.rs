use super::residue::read_csv_records;
use crate::{
    RocketConstants,
    state_estimator::{
        ascent_fusion::mekf::{State, state_propagation::state_transition},
        welford::Welford,
    },
    tests::{init_logger, plot::GlobalPlot},
};

#[test]
fn calculate_process_noise() {
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

    let mut welford = Welford::<{ State::SIZE }>::new();
    for (csv_record, next_record) in current_records.zip(next_records) {
        GlobalPlot::set_time_s(csv_record.timestamp_s);

        let state = csv_record.to_rocket_state();
        let orientation = csv_record.to_orientation();

        let predicted_state = state_transition(0.0, &orientation, &state, &constants);

        let mut true_state = next_record.to_rocket_state();
        let true_orientation = next_record.to_orientation();
        let true_small_angle_correction = -(orientation * true_orientation.inverse()).scaled_axis();
        true_state.0[0] = true_small_angle_correction[0];
        true_state.0[1] = true_small_angle_correction[1];
        true_state.0[2] = true_small_angle_correction[2];
        

        welford.update(&(true_state.0 - predicted_state.0));
    }
    GlobalPlot::plot_all();
    log_info!("{:?}", welford.variance().unwrap());
}
