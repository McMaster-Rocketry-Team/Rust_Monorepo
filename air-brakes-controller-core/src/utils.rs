/// Air density (kg/m^3) at altitude ASL (m), ISA troposphere formula
/// (valid to 11 km).
///
/// The full formula rather than a low-altitude linear fit: LC'26 simulates
/// to 6+ km, and under-reading density over-predicts apogee, which
/// over-extends the airbrakes — a one-sided error in the harmful direction.
///
/// `libm::powf` by name, not `x.powf(y)`. The method form resolves to the
/// inherent `f32::powf` under std and to whatever `F32Ext` trait is in
/// scope under `no_std` — so the same line computed different numbers on
/// the host and on the board. With `micromath` in scope that cost 39% of
/// the density at 10 km, which inflated the Mach-lockout exit's drag
/// inversion by 28% and delayed the exit by 3 s on the bench (measured,
/// 2026-08-16). A free function has no such resolution, so the tests and
/// the rocket now run the same arithmetic.
pub fn approximate_air_density(altitude_asl: f32) -> f32 {
    1.225 * libm::powf((1.0 - 2.25577e-5 * altitude_asl).max(0.0), 4.256)
}

/// Speed of sound (m/s) at altitude ASL (m): linear fit to the standard
/// atmosphere, within 0.3% of ISA up to 8 km (sound speed really is
/// near-linear in the troposphere, unlike density).
pub fn approximate_speed_of_sound(altitude_asl: f32) -> f32 {
    340.29 - altitude_asl * 0.003903
}

pub fn lerp(
    t: f32, // 0-1
    values: &[f32],
) -> f32 {
    let len = values.len();
    let spacing = 1.0f32 / ((len - 1) as f32);

    let mut i = (t / spacing) as usize;
    if i > len - 2 {
        i = len - 2;
    }

    let t = (t - spacing * (i as f32)) * (len - 1) as f32;
    (1.0 - t) * values[i] + t * values[i + 1]
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;

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


}