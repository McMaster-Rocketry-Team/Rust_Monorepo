/// returns air density (kg/m^3) and speed of sound (m/s) at altitude (m)
/// approximated using a linear function from 0m and 3000m data from standard atmosphere model
pub fn approximate_air_density(altitude_asl: f32) -> f32 {
    1.225 - altitude_asl * 0.0001053
}


/// returns air density (kg/m^3) and speed of sound (m/s) at altitude (m)
/// approximated using a linear function from 0m and 3000m data from standard atmosphere model
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