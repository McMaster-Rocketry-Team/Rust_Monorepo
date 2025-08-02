use nalgebra::{
    Const, Matrix, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3, Vector4, ViewStorage,
};

pub struct RocketState(pub SVector<f32, 18>);

type VectorView<'a, const R: usize> =
    Matrix<f32, Const<R>, Const<1>, ViewStorage<'a, f32, Const<R>, Const<1>, Const<1>, Const<18>>>;

// ENU
impl RocketState {
    pub fn new(
        small_angle_correction: &Vector3<f32>,
        acceleration: &Vector3<f32>,
        velocity: &Vector3<f32>,
        angular_velocity: &Vector3<f32>,
        altitude_agl: f32,
        sideways_moment_co: f32,
        drag_coefficients: &Vector4<f32>,
    ) -> Self {
        let mut state = SVector::zeros();

        // Small angle correction (indices 0-2)
        state[0] = small_angle_correction[0];
        state[1] = small_angle_correction[1];
        state[2] = small_angle_correction[2];

        // Acceleration (indices 3-5)
        state[3] = acceleration[0];
        state[4] = acceleration[1];
        state[5] = acceleration[2];

        // Velocity (indices 6-8)
        state[6] = velocity[0];
        state[7] = velocity[1];
        state[8] = velocity[2];

        // Angular velocity (indices 9-11)
        state[9] = angular_velocity[0];
        state[10] = angular_velocity[1];
        state[11] = angular_velocity[2];

        // Altitude AGL (index 12)
        state[12] = altitude_agl;

        // Sideways moment coefficient (index 13)
        state[13] = sideways_moment_co;

        // Drag coefficients (indices 14-17)
        state[14] = drag_coefficients[0];
        state[15] = drag_coefficients[1];
        state[16] = drag_coefficients[2];
        state[17] = drag_coefficients[3];

        RocketState(state)
    }

    pub fn small_angle_correction(&self) -> VectorView<'_, 3> {
        self.0.fixed_view::<3, 1>(0, 0)
    }

    pub fn acceleration(&self) -> VectorView<'_, 3> {
        self.0.fixed_view::<3, 1>(3, 0)
    }

    pub fn velocity(&self) -> VectorView<'_, 3> {
        self.0.fixed_view::<3, 1>(6, 0)
    }

    pub fn angular_velocity(&self) -> VectorView<'_, 3> {
        self.0.fixed_view::<3, 1>(9, 0)
    }

    pub fn altitude_agl(&self) -> f32 {
        self.0[12]
    }

    pub fn sideways_moment_co(&self) -> f32 {
        self.0[13]
    }

    pub fn drag_coefficients(&self) -> VectorView<'_, 4> {
        self.0.fixed_view::<4, 1>(14, 0)
    }

    pub fn add_derivative(&self, d: &Derivative<RocketState>, dt: f32) -> RocketState {
        let mut new_state = self.0.clone();

        // Add derivative scaled by dt to each component
        for i in 0..18 {
            new_state[i] += d.0.0[i] * dt;
        }

        RocketState(new_state)
    }

    /// apply small angle correction to the supplied quaternion and
    /// reset small angle correction to zero
    pub fn reset_small_angle_correction(&mut self, orientation: &UnitQuaternion<f32>)->UnitQuaternion<f32> {
        let delta_orientation = UnitQuaternion::from_quaternion(Quaternion::from_parts(
            1.0,
            -self.small_angle_correction() / 2.0,
        ));

        self.0[0] = 0.0;
        self.0[1] = 0.0;
        self.0[2] = 0.0;

        delta_orientation * *orientation
    }
}

pub struct Derivative<T>(pub T);

pub struct StateDerivativeConstants {
    pub launch_site_altitude_asl: f32,
    pub side_cd: f32,
    pub burn_out_mass: f32,
    pub moment_of_inertia: f32,
    pub front_reference_area: f32,
    pub side_reference_area: f32,
}

/// returns air density (kg/m^3) and speed of sound (m/s) at altitude (m)
/// approximated using a linear function from 0m and 3000m data from standard atmosphere model
pub fn approximate_air_density(altitude_asl: f32) -> (f32, f32) {
    let air_density = 1.225 - altitude_asl * 0.0001053;
    let speed_of_sound = 340.29 - altitude_asl * 0.003903;

    (air_density, speed_of_sound)
}

fn lerp(
    t: f32, // 0-1
    drag_coefficients: &[f32],
) -> f32 {
    let len = drag_coefficients.len();
    let spacing = 1.0f32 / ((len - 1) as f32);

    let mut i = (t / spacing) as usize;
    if i > len - 2 {
        i = len - 2;
    }

    let t = (t - spacing * (i as f32)) * (len - 1) as f32;
    (1.0 - t) * drag_coefficients[i] + t * drag_coefficients[i + 1]
}

pub fn calculate_state_derivative(
    airbrakes_extention: f32, // 0-1
    orientation: &UnitQuaternion<f32>,
    state: &RocketState,
    constants: &StateDerivativeConstants,
) -> Derivative<RocketState> {
    let altitude_asl = state.altitude_agl() + constants.launch_site_altitude_asl;
    let (air_density, _) = approximate_air_density(altitude_asl);

    let delta_orientation = UnitQuaternion::from_quaternion(Quaternion::from_parts(
        1.0,
        -state.small_angle_correction() / 2.0,
    ));
    let true_orientation = delta_orientation * orientation;

    let wind_vel_rocket_frame =
        -true_orientation.inverse_transform_vector(&state.velocity().into());

    // calculate drag coefficient
    let forward_cd = lerp(airbrakes_extention, state.drag_coefficients().as_slice());
    let cd = Vector3::new(constants.side_cd, constants.side_cd, forward_cd);
    let reference_area = Vector3::new(
        constants.side_reference_area,
        constants.side_reference_area,
        constants.front_reference_area,
    );

    let d_acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
        * 2.0
        * wind_vel_rocket_frame
            .abs()
            .component_mul(&cd)
            .component_mul(&reference_area);
    let d_acc_world_frame = true_orientation.transform_vector(&d_acc_rocket_frame);

    // also depend on state.acceleration();
    // let acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
    //     * wind_vel_rocket_frame
    //         .component_mul(&wind_vel_rocket_frame.abs())
    //         .component_mul(&cd);
    // let mut acc_world_frame = true_orientation.transform_vector(&acc_rocket_frame);
    // acc_world_frame.z -= 9.81;

    let mut angular_acceleration_rocket_frame = Vector3::<f32>::zeros();
    angular_acceleration_rocket_frame.x = 0.5 * air_density / constants.moment_of_inertia
        * constants.side_reference_area
        * wind_vel_rocket_frame.y
        * wind_vel_rocket_frame.y.abs()
        * state.sideways_moment_co();
    angular_acceleration_rocket_frame.y = -0.5 * air_density / constants.moment_of_inertia
        * constants.side_reference_area
        * wind_vel_rocket_frame.x
        * wind_vel_rocket_frame.x.abs()
        * state.sideways_moment_co();

    let angular_acceleration_world_frame =
        true_orientation.transform_vector(&angular_acceleration_rocket_frame);

    let angular_velocity_rocket_frame =
        true_orientation.inverse_transform_vector(&state.angular_velocity().into());

    Derivative(RocketState::new(
        &angular_velocity_rocket_frame,
        &d_acc_world_frame,
        &state.acceleration().into(),
        &angular_acceleration_world_frame,
        state.velocity().z,
        0.0,
        &Vector4::zeros(),
    ))
}

pub fn central_difference_jacobian(
    airbrakes_ext: f32,
    orientation: UnitQuaternion<f32>,
    state: &RocketState,
    constants: &StateDerivativeConstants,
) -> SMatrix<f32, 18, 18> {
    let x0 = state.0;

    let mut j_mat = SMatrix::<f32, 18, 18>::zeros();

    for j in 0..x0.len() {
        // TODO tune
        let delta = match j {
            0..3 => 0.1f32.to_radians(),
            3..6 => 0.1f32,
            6..9 => 0.1f32.max(x0[j].abs() * 0.001),
            9..12 => 1f32.to_radians(),
            12 => 0.5f32,
            13 => 0.001f32,
            14..18 => 0.01f32,
            _ => 0f32,
        };

        // x+δ
        let mut x_plus = x0;
        x_plus[j] += delta;
        let f_plus = calculate_state_derivative(
            airbrakes_ext,
            &orientation,
            &RocketState(x_plus),
            constants,
        );
        let f_plus_vec = f_plus.0.0;

        // x-δ
        let mut x_minus = x0;
        x_minus[j] -= delta;
        let f_minus = calculate_state_derivative(
            airbrakes_ext,
            &orientation,
            &RocketState(x_minus),
            constants,
        );
        let f_minus_vec = f_minus.0.0;

        // central difference: (f+ − f−) / (2δ)
        let column = (f_plus_vec - f_minus_vec) / (2.0 * delta);
        j_mat.set_column(j, &column);
    }

    j_mat
}

pub fn build_measurement_matrix() -> SMatrix<f32, 7, 18> {
    let mut h = SMatrix::zeros();

    // ---- acceleration rows (rows 0-2) ----
    h[(0, 3)] = 1.0; // ax   ← state[3]
    h[(1, 4)] = 1.0; // ay   ← state[4]
    h[(2, 5)] = 1.0; // az   ← state[5]

    // ---- angular-rate rows (rows 3-5) ----
    h[(3, 9)] = 1.0; // ωx   ← state[9]
    h[(4, 10)] = 1.0; // ωy   ← state[10]
    h[(5, 11)] = 1.0; // ωz   ← state[11]

    // ---- altitude row (row 6) ----
    h[(6, 12)] = 1.0; // h    ← state[12]

    h
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use nalgebra::UnitVector3;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn lerp_test() {
        assert_relative_eq!(
            lerp(-1f32 / 3.0, &[0.0, 1.0, 2.0, 3.0]),
            -1.0,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(0.0f32, &[0.0, 1.0, 2.0, 3.0]), 0.0, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(0.16666666f32, &[0.0, 1.0, 2.0, 3.0]),
            0.5,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(0.5f32, &[0.0, 1.0, 2.0, 3.0]), 1.5, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(0.83333333f32, &[0.0, 1.0, 2.0, 3.0]),
            2.5,
            epsilon = 0.0001
        );
        assert_relative_eq!(lerp(1.0f32, &[0.0, 1.0, 2.0, 3.0]), 3.0, epsilon = 0.0001);
        assert_relative_eq!(
            lerp(1.0f32 + 1.0 / 3.0, &[0.0, 1.0, 2.0, 3.0]),
            4.0,
            epsilon = 0.0001
        );
    }

    #[test]
    fn small_angle_correction_test() {
        init_logger();

        fn euler_angles_to_degrees(radians: (f32, f32, f32)) -> [f32; 3] {
            [
                radians.0.to_degrees(),
                radians.1.to_degrees(),
                radians.2.to_degrees(),
            ]
        }

        let small_angle = Vector3::new(0f32.to_radians(), 0f32.to_radians(), 1f32.to_radians());
        let delta_orientation_quaternion: UnitQuaternion<f32> =
            UnitQuaternion::from_quaternion(Quaternion::from_parts(1.0, -small_angle / 2.0));

        let mut orientation = UnitQuaternion::<f32>::identity();
        orientation = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_normalize(Vector3::new(1f32, 0f32, 0f32)),
            90f32.to_radians(),
        );
        log_info!(
            "initial orientation: {:?}",
            euler_angles_to_degrees(orientation.euler_angles())
        );
        orientation = delta_orientation_quaternion * orientation;

        log_info!(
            "rotated orientation: {:?}",
            euler_angles_to_degrees(orientation.euler_angles())
        );
    }
}
