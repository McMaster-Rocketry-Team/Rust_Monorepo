use nalgebra::{SVector, UnitQuaternion, Vector3, Vector4};

use crate::{utils::{approximate_air_density, lerp}, RocketConstants};

pub struct State(pub SVector<f32, { Self::SIZE }>);

/// ENU, all in earth frame
impl State {
    pub const SIZE: usize = 15;

    pub fn new(
        small_angle_correction: &Vector3<f32>,
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

        // Velocity (indices 3-5)
        state[3] = velocity[0];
        state[4] = velocity[1];
        state[5] = velocity[2];

        // Angular velocity (indices 6-9)
        state[6] = angular_velocity[0];
        state[7] = angular_velocity[1];
        state[8] = angular_velocity[2];

        // Altitude AGL (index 9)
        state[9] = altitude_agl;

        // Sideways moment coefficient (index 10)
        state[10] = sideways_moment_co;

        // Drag coefficients (indices 11-15)
        state[11] = drag_coefficients[0];
        state[12] = drag_coefficients[1];
        state[13] = drag_coefficients[2];
        state[14] = drag_coefficients[3];

        State(state)
    }

    pub fn small_angle_correction(&self) -> Vector3<f32> {
        self.0.fixed_view::<3, 1>(0, 0).into()
    }

    /// Expected specific force in earth frame (non-gravitational acceleration):
    /// transforms aerodynamic acceleration from body to earth frame.
    pub fn expected_acceleration(
        &self,
        airbrakes_extention: f32,
        orientation: &UnitQuaternion<f32>,
        constants: &RocketConstants,
    ) -> Vector3<f32> {
        let air_density = approximate_air_density(self.altitude_asl());
        let delta_orientation = UnitQuaternion::from_scaled_axis(self.small_angle_correction());
        let true_orientation = orientation * delta_orientation;

        let wind_vel_rocket_frame =
            -true_orientation.inverse_transform_vector(&self.velocity().into());

        let forward_cd = lerp(airbrakes_extention, self.drag_coefficients().as_slice());
        let cd = Vector3::new(constants.side_cd, constants.side_cd, forward_cd);
        let reference_area = Vector3::new(
            constants.side_reference_area,
            constants.side_reference_area,
            constants.front_reference_area,
        );

        let acc_rocket_frame = 0.5 * air_density / constants.burn_out_mass
            * wind_vel_rocket_frame
                .component_mul(&wind_vel_rocket_frame.abs())
                .component_mul(&cd)
                .component_mul(&reference_area);
        true_orientation.transform_vector(&acc_rocket_frame)
    }

    pub fn velocity(&self) -> Vector3<f32> {
        self.0.fixed_view::<3, 1>(3, 0).into()
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        self.0.fixed_view::<3, 1>(6, 0).into()
    }

    pub fn altitude_asl(&self) -> f32 {
        self.0[9]
    }

    pub fn sideways_moment_co(&self) -> f32 {
        self.0[10]
    }

    pub fn drag_coefficients(&self) -> Vector4<f32> {
        self.0.fixed_view::<4, 1>(11, 0).into()
    }

    /// apply small angle correction to the supplied quaternion and
    /// reset small angle correction to zero
    pub fn reset_small_angle_correction(
        &mut self,
        orientation: &UnitQuaternion<f32>,
    ) -> UnitQuaternion<f32> {
        // TODO double check
        let delta_orientation = UnitQuaternion::from_scaled_axis(self.small_angle_correction());

        self.0[0] = 0.0;
        self.0[1] = 0.0;
        self.0[2] = 0.0;

        *orientation * delta_orientation
    }
}
