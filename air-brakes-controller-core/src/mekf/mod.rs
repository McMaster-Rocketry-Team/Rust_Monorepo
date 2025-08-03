use nalgebra::{Const, Matrix, Quaternion, SVector, UnitQuaternion, Vector3, Vector4, ViewStorage};

pub mod mekf;
pub mod state_propagation;
#[cfg(test)]
mod tests;

pub struct RocketConstants {
    pub side_cd: f32,
    pub burn_out_mass: f32,
    pub moment_of_inertia: f32,
    pub front_reference_area: f32,
    pub side_reference_area: f32,
}

pub struct Derivative<T>(pub T);

pub struct State(pub SVector<f32, { Self::SIZE }>);

type VectorView<'a, const R: usize, const N: usize> =
    Matrix<f32, Const<R>, Const<1>, ViewStorage<'a, f32, Const<R>, Const<1>, Const<1>, Const<N>>>;

/// ENU
impl State {
    pub const SIZE: usize = 18;

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

        State(state)
    }

    pub fn small_angle_correction(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(0, 0)
    }

    pub fn acceleration(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(3, 0)
    }

    pub fn velocity(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(6, 0)
    }

    pub fn angular_velocity(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(9, 0)
    }

    pub fn altitude_asl(&self) -> f32 {
        self.0[12]
    }

    pub fn sideways_moment_co(&self) -> f32 {
        self.0[13]
    }

    pub fn drag_coefficients(&self) -> VectorView<'_, 4, { Self::SIZE }> {
        self.0.fixed_view::<4, 1>(14, 0)
    }

    pub fn add_derivative(&self, d: &Derivative<State>, dt: f32) -> State {
        let mut new_state = self.0.clone();

        for i in 0..Self::SIZE {
            new_state[i] += d.0.0[i] * dt;
        }

        State(new_state)
    }

    /// apply small angle correction to the supplied quaternion and
    /// reset small angle correction to zero
    pub fn reset_small_angle_correction(
        &mut self,
        orientation: &UnitQuaternion<f32>,
    ) -> UnitQuaternion<f32> {
        let delta_orientation = UnitQuaternion::from_quaternion(Quaternion::from_parts(
            1.0,
            self.small_angle_correction() / 2.0, // TODO double check
        ));

        self.0[0] = 0.0;
        self.0[1] = 0.0;
        self.0[2] = 0.0;

        delta_orientation * *orientation
    }
}

/// Values are in rocket body frame
#[derive(Debug, Clone)]
pub struct Measurement(pub SVector<f32, { Self::SIZE }>);

impl Measurement {
    pub const SIZE: usize = 7;

    pub fn new(
        acceleration: &Vector3<f32>,
        angular_velocity: &Vector3<f32>,
        altitude_asl: f32,
    ) -> Self {
        let mut vec = SVector::<f32, 7>::zeros();
        vec.fixed_view_mut::<3, 1>(0, 0).copy_from(acceleration);
        vec.fixed_view_mut::<3, 1>(3, 0).copy_from(angular_velocity);
        vec[6] = altitude_asl;
        Self(vec)
    }

    pub fn acceleration(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(0, 0)
    }

    pub fn angular_velocity(&self) -> VectorView<'_, 3, { Self::SIZE }> {
        self.0.fixed_view::<3, 1>(3, 0)
    }

    pub fn altitude_asl(&self) -> f32 {
        self.0[6]
    }
}
