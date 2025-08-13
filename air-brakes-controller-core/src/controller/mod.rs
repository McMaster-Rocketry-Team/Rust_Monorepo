use nalgebra::Vector2;

use crate::{controller::rocket_dynamics::simulate_apogee_rk2, utils::lerp};

const DT: f32 = 0.1;

mod rocket_dynamics;

pub struct AirBrakesMPC {
    parameters: RocketParameters,
    target_apogee_asl: f32,
}

impl AirBrakesMPC {
    pub fn new(parameters: RocketParameters, target_apogee_asl: f32) -> Self {
        Self {
            parameters,
            target_apogee_asl,
        }
    }

    /// returns air brakes extension percentage 0.0 - 1.0
    pub fn update(&mut self, current_altitude_asl: f32, current_velocity: Vector2<f32>) -> f32 {
        // Build initial state
        let initial_state = State {
            altitude_asl: current_altitude_asl,
            velocity: current_velocity,
        };

        // Search interval for drag percentage [-1.0, 1.0]
        let mut low_drag = -1.0f32;
        let mut high_drag = 1.0f32;

        let mut ap_low = simulate_apogee_rk2(low_drag, &initial_state, &self.parameters);
        let mut ap_high = simulate_apogee_rk2(high_drag, &initial_state, &self.parameters);

        // Perform up to 3 iterations of bisection
        for _ in 0..3 {
            let mid_drag = 0.5 * (low_drag + high_drag);
            let ap_mid = simulate_apogee_rk2(mid_drag, &initial_state, &self.parameters);

            // Monotonic: higher drag -> lower apogee
            if ap_mid > self.target_apogee_asl {
                // Need more drag to reduce apogee
                low_drag = mid_drag;
                ap_low = ap_mid;
            } else {
                // Too much drag, reduce it
                high_drag = mid_drag;
                ap_high = ap_mid;
            }
        }

        // After 3 iterations, linearly interpolate between the bracket endpoints
        // ap_low corresponds to low_drag (higher apogee), ap_high to high_drag (lower apogee)
        let denom = ap_low - ap_high;
        let t = if denom.abs() < 1e-6 {
            0.5
        } else {
            ((self.target_apogee_asl - ap_high) / denom).clamp(0.0, 1.0)
        };
        let drag_percentage = high_drag + t * (low_drag - high_drag);

        // Convert to extension percentage and clamp to [0,1]
        self.parameters
            .drag_percentage_to_extension_percentage(drag_percentage)
    }
}

#[derive(Debug, Clone)]
struct State {
    altitude_asl: f32,
    velocity: Vector2<f32>,
}

struct Derivative<T>(T);

#[derive(Clone, Debug)]
pub struct RocketParameters {
    pub burnout_mass: f32,
    /// cd is a look up table from extension percentage to cd
    /// e.g. cd[2] is cd at 50% extension percentage
    pub cd: [f32; 5],
    pub reference_area: f32,
}

impl RocketParameters {
    /// drag percentage: -1.0 - 1.0
    fn get_cd_from_drag_percentage(&self, drag_percentage: f32) -> f32 {
        lerp(
            (drag_percentage + 1.0) / 2.0,
            &[self.cd[0], self.cd[self.cd.len() - 1]],
        )
    }

    /// drag percentage: -1.0 - 1.0
    /// returns 0.0 to 1.0
    fn drag_percentage_to_extension_percentage(&self, drag_percentage: f32) -> f32 {
        let cd = self.get_cd_from_drag_percentage(drag_percentage);

        // cd is strictly increasing; map back to extension percentage in [0,1]
        let n = self.cd.len();

        let first = self.cd[0];
        let last = self.cd[n - 1];
        let cd = cd.clamp(first, last);

        if cd <= first {
            return 0.0;
        }
        if cd >= last {
            return 1.0;
        }

        let segment_width = 1.0 / (n as f32 - 1.0);
        for i in 0..(n - 1) {
            let a = self.cd[i];
            let b = self.cd[i + 1];
            // Since strictly increasing, cd will be <= b at the matching segment
            if cd <= b {
                let t = (cd - a) / (b - a);
                return (i as f32 + t) * segment_width;
            }
        }

        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::RocketParameters;
    use approx::assert_relative_eq;

    fn params() -> RocketParameters {
        RocketParameters {
            burnout_mass: 10.0,
            cd: [0.3, 0.4, 0.5, 0.65, 0.8],
            reference_area: 0.02,
        }
    }

    #[test]
    fn maps_ends_correctly() {
        let p = params();
        // drag -1 -> cd at start -> extension 0
        let e0 = p.drag_percentage_to_extension_percentage(-1.0);
        assert_relative_eq!(e0, 0.0, epsilon = 1e-6);

        // drag +1 -> cd at end -> extension 1
        let e1 = p.drag_percentage_to_extension_percentage(1.0);
        assert_relative_eq!(e1, 1.0, max_relative = 1e-6);
    }

    #[test]
    fn clamps_out_of_range() {
        let p = params();
        let e_low = p.drag_percentage_to_extension_percentage(-10.0);
        let e_high = p.drag_percentage_to_extension_percentage(10.0);
        assert_relative_eq!(e_low, 0.0, epsilon = 1e-6);
        assert_relative_eq!(e_high, 1.0, max_relative = 1e-6);
    }

    #[test]
    fn interpolates_linearly_across_segments() {
        let p = params();

        // drag 0 maps to cd midpoint between ends: (0.3 + 0.8)/2 = 0.55
        // In the table: [0.5, 0.65] segment (indices 2..3)
        // t = (0.55 - 0.5) / (0.65 - 0.5) = 0.3333333
        // segment width = 1/(5-1) = 0.25
        // extension = (2 + t) * 0.25 = (2.3333333) * 0.25 = 0.5833333
        let e_mid = p.drag_percentage_to_extension_percentage(0.0);
        assert_relative_eq!(e_mid, 0.5833333, max_relative = 1e-5);

        // Another point near start: drag -0.5 -> cd = lerp(0.25, [0.3, 0.8]) = 0.425
        // Segment [0.4, 0.5] (indices 1..2)
        // t = (0.425 - 0.4) / (0.5 - 0.4) = 0.25
        // ext = (1 + 0.25) * 0.25 = 0.3125
        let e_q1 = p.drag_percentage_to_extension_percentage(-0.5);
        assert_relative_eq!(e_q1, 0.3125, max_relative = 1e-6);
    }
}
