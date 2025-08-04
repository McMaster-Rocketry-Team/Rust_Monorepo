use core::hint::black_box;

use nalgebra::{UnitQuaternion, UnitVector3, Vector3, Vector4};

use crate::{
    tests::{init_logger, to_matlab}, RocketConstants,
};

use super::super::state::State;
use super::super::state_propagation::{build_measurement_matrix, central_difference_jacobian};

#[test]
fn jacobian_benchmark() {
    use std::time::Instant;

    init_logger();

    // Create arbitrary but realistic test data
    let airbrakes_ext = 0.5f32; // 50% extension

    let orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::new(0.1, 0.2, 0.9)),
        15f32.to_radians(),
    );

    let state = State::new(
        &Vector3::new(0.01, -0.02, 0.005), // small angle correction (radians)
        &Vector3::new(0.0, -1.5, 0.0),     // acceleration (m/s^2)
        &Vector3::new(2.0, 100.0, 45.0),   // velocity (m/s)
        &Vector3::new(0.1, -0.05, 0.02),   // angular velocity (rad/s)
        1200.0,                            // altitude AGL (m)
        0.8,                               // sideways moment coefficient
        &Vector4::new(0.4, 0.6, 0.8, 1.0), // drag coefficients
    );

    let constants = RocketConstants {
        side_cd: 0.02,          // side drag coefficient
        burn_out_mass: 25.0,    // mass (kg)
        moment_of_inertia: 2.5, // moment of inertia (kg⋅m²)
        front_reference_area: 0.01368,
        side_reference_area: 0.3575,
    };

    // Benchmark multiple runs
    let num_runs = 100;
    let start_time = Instant::now();

    for _ in 0..num_runs {
        let _ = black_box(central_difference_jacobian(
            black_box(airbrakes_ext),
            &black_box(orientation),
            black_box(&state),
            black_box(&constants),
        ));
    }

    let total_duration = start_time.elapsed();
    let avg_duration = total_duration / num_runs;

    log_info!("Jacobian computation benchmark:");
    log_info!("  Total time for {} runs: {:?}", num_runs, total_duration);
    log_info!("  Average time per run: {:?}", avg_duration);
    log_info!(
        "  Average time per run (microseconds): {:.2}",
        avg_duration.as_micros()
    );

    log_info!("jacobian:");
    log_info!(
        "A={};",
        to_matlab(&central_difference_jacobian(
            airbrakes_ext,
            &orientation,
            &state,
            &constants
        ))
    );
    log_info!("measurement matrix:");
    log_info!("H={};", to_matlab(&build_measurement_matrix()));
}
