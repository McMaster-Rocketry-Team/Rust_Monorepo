use air_brakes_controller_core::{AirBrakesMPC, RocketParameters};
use nalgebra::Vector2;

#[unsafe(no_mangle)]
pub extern "C" fn get_air_brakes_extension_percentage(
    burnout_mass: f32,
    cd_0: f32,
    cd_25: f32,
    cd_50: f32,
    cd_75: f32,
    cd_100: f32,
    reference_area: f32,
    target_apogee_asl: f32,
    current_altitude_asl: f32,
    current_vertical_velocity: f32,
) -> f32 {
    let cd = [cd_0, cd_25, cd_50, cd_75, cd_100];
    let rocket_parameters = RocketParameters {
        burnout_mass,
        cd,
        reference_area,
    };

    let airbrakes_mpc = AirBrakesMPC::new(rocket_parameters, target_apogee_asl);

    // The core solver takes (horizontal, vertical) since it models drag on the
    // full velocity vector. This entry point only receives the vertical
    // component, so horizontal is 0.0 — which is what the solver saw back when
    // `update` took a scalar, so behaviour through this export is unchanged.
    // Widening the export to carry horizontal velocity would be an ABI change.
    airbrakes_mpc
        .update(
            current_altitude_asl,
            Vector2::new(0.0, current_vertical_velocity),
        )
        .extension_percentage
}
