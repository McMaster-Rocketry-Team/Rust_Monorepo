use crate::{
    controller::{DT, Derivative, RocketParameters, State},
    utils::approximate_air_density,
};

pub fn calculate_state_derivatives(
    air_brakes_drag_percentage: f32,
    state: &State,
    rocket_param: &RocketParameters,
) -> Derivative<State> {
    let air_density = approximate_air_density(state.altitude_asl);

    let speed_squared = state.velocity.magnitude_squared();
    let cd = rocket_param.get_cd_from_drag_percentage(air_brakes_drag_percentage);
    let drag_force = 0.5 * cd * air_density * speed_squared * rocket_param.reference_area;
    let acceleration = drag_force / rocket_param.burnout_mass;
    let mut acceleration = -state.velocity.normalize() * acceleration;
    acceleration.y -= 9.81;

    Derivative(State {
        altitude_asl: state.velocity.y,
        velocity: acceleration,
    })
}

// use rk2 to simulate the rocket until apogee
// apogee is when the vertical velocity <= 0
// in the first timestep, use first_tick_air_brakes_extension
// in all the following timestep, use 0.0 as air brakes extension
pub fn simulate_apogee_rk2(
    first_tick_air_brakes_drag_percentage: f32,
    initial_state: &State,
    rocket_param: &RocketParameters,
) -> f32 {
    // If we are already descending or stationary, return current altitude
    if initial_state.velocity.y <= 0.0 {
        return initial_state.altitude_asl;
    }

    let mut state = initial_state.clone();
    let mut step_index: usize = 0;

    loop {
        let air_brakes_drag_percentage = match step_index {
            0 => first_tick_air_brakes_drag_percentage,
            1 => first_tick_air_brakes_drag_percentage / 2.0,
            _ => 0.0,
        };

        // RK2 (midpoint) integration
        let Derivative(k1) =
            calculate_state_derivatives(air_brakes_drag_percentage, &state, rocket_param);

        let mid_state = State {
            altitude_asl: state.altitude_asl + k1.altitude_asl * (0.5 * DT),
            velocity: state.velocity + k1.velocity * (0.5 * DT),
        };

        let Derivative(k2) =
            calculate_state_derivatives(air_brakes_drag_percentage, &mid_state, rocket_param);

        let next_state = State {
            altitude_asl: state.altitude_asl + k2.altitude_asl * DT,
            velocity: state.velocity + k2.velocity * DT,
        };

        // Check for apogee crossing within this step
        let vy0 = state.velocity.y;
        let vy1 = next_state.velocity.y;
        if vy1 <= 0.0 {
            // Linearly interpolate vertical velocity over the step to estimate
            // the exact time t_zero where v_y crosses zero, then integrate
            // velocity to get altitude at apogee.
            let denom = vy1 - vy0;
            if denom.abs() < core::f32::EPSILON {
                return next_state.altitude_asl.max(state.altitude_asl);
            }
            let t_zero = DT * (-vy0) / denom; // 0 <= t_zero <= DT
            let delta_alt = vy0 * t_zero + 0.5 * (denom / DT) * t_zero * t_zero;
            return state.altitude_asl + delta_alt;
        }

        state = next_state;
        step_index += 1;
    }
}

#[cfg(test)]
mod test {
    use nalgebra::Vector2;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_simulate_apogee() {
        init_logger();

        let initial_state = State {
            altitude_asl: 1032.0 + 251.0,
            velocity: Vector2::new(66.8630616, 308.7624),
        };

        let rocket_param = RocketParameters {
            burnout_mass: 19.417,
            cd: [0.5; 5],
            reference_area: 0.0136,
        };

        log_info!(
            "simulated apogee: {}",
            simulate_apogee_rk2(0.5, &initial_state, &rocket_param)
        );
    }

    #[test]
    fn bench_simulate_apogee_rk2_100x() {
        use core::hint::black_box;

        init_logger();

        let initial_state = State {
            altitude_asl: 1032.0 + 251.0,
            velocity: Vector2::new(66.8630616, 308.7624),
        };

        let rocket_param = RocketParameters {
            burnout_mass: 19.417,
            cd: [0.5; 5],
            reference_area: 0.0136,
        };

        let start = std::time::Instant::now();
        let mut sum = 0.0f32;
        for _ in 0..100 {
            let apogee = simulate_apogee_rk2(
                black_box(0.5f32),
                black_box(&initial_state),
                black_box(&rocket_param),
            );
            sum += apogee;
        }
        let elapsed = start.elapsed();

        log_info!(
            "bench simulate_apogee_rk2: {:?} each, result: {}",
            elapsed / 100,
            sum / 100.0
        );
    }
}
