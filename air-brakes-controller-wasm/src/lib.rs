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
    current_horizontal_velocity: f32,
    current_vertical_velocity: f32,
) -> f32 {
    let cd = [cd_0, cd_25, cd_50, cd_75, cd_100];
    let rocket_parameters = RocketParameters {
        burnout_mass,
        cd,
        reference_area,
    };

    let mut airbrakes_mpc = AirBrakesMPC::new(rocket_parameters, target_apogee_asl);

    airbrakes_mpc.update(
        current_altitude_asl,
        Vector2::new(current_horizontal_velocity, current_vertical_velocity),
    )
}
