#![allow(dead_code)]

#[macro_export]
macro_rules! fixed_point_factory {
    ($name:ident, f32, $min:literal, $max:literal, $max_error:literal) => {
        fixed_point_factory!($name, minmax, f32, libm::roundf, $min, $max, $max_error);
    };
    ($name:ident, f64, $min:literal, $max:literal, $max_error:literal) => {
        fixed_point_factory!($name, minmax, f64, libm::round, $min, $max, $max_error);
    };
    ($name:ident, $mode: ident, $source: ty, $round_fn: path, $min:literal, $max:literal, $max_error:literal) => {
        calculate_required_bits::calculate_required_bits_docstr!(
            $mode, $min, $max, $max_error, $name
        );

        paste::paste! {
            type [<$name Packed>] = calculate_required_bits::calculate_packed_type!($mode, $min, $max, $max_error);
            type [<$name Base>] = calculate_required_bits::calculate_base_type!($mode, $min, $max, $max_error);

            impl $name {
                fn to_fixed_point(value: $source) -> Option<[<$name Packed>]> {
                    if value < calculate_required_bits::calculate_min!($mode, $min, $max, $max_error) as $source || value > calculate_required_bits::calculate_max!($mode, $min, $max, $max_error) as $source{
                        return None;
                    }
                    let value = value - calculate_required_bits::calculate_min!($mode, $min, $max, $max_error)as $source;
                    let value = value / (calculate_required_bits::calculate_max!($mode, $min, $max, $max_error)as $source - calculate_required_bits::calculate_min!($mode, $min, $max, $max_error)as $source);
                    let value = value * Self::_target_max() as $source;
                    Some(
                        num_traits::cast::<$source, [<$name Base>]>($round_fn(value))
                            .unwrap()
                            .into(),
                    )
                }

                fn to_fixed_point_capped(value: $source) -> [<$name Packed>] {
                    let value = if value < calculate_required_bits::calculate_min!($mode, $min, $max, $max_error) as $source{
                        calculate_required_bits::calculate_min!($mode, $min, $max, $max_error)as $source
                    } else if value > calculate_required_bits::calculate_max!($mode, $min, $max, $max_error)as $source {
                        calculate_required_bits::calculate_max!($mode, $min, $max, $max_error)as $source
                    } else {
                        value
                    };
                    return Self::to_fixed_point(value).unwrap();
                }

                fn to_float(value: [<$name Packed>]) -> $source {
                    let value: [<$name Base>] = value.into();
                    let value = value as $source;
                    let value = value / Self::_target_max() as $source;
                    let value = value * (calculate_required_bits::calculate_max!($mode, $min, $max, $max_error) as $source- calculate_required_bits::calculate_min!($mode, $min, $max, $max_error)as $source);
                    value + calculate_required_bits::calculate_min!($mode, $min, $max, $max_error)as $source
                }

                fn max() -> $source {
                    calculate_required_bits::calculate_max!($mode, $min, $max, $max_error) as $source
                }

                fn min() -> $source {
                    calculate_required_bits::calculate_min!($mode, $min, $max, $max_error) as $source
                }
            }

            impl $name {
                fn _target_max() -> [<$name Base>] {
                    num_traits::cast::<u8, [<$name Base>]>(1)
                        .unwrap()
                        .checked_shl(calculate_required_bits::calculate_required_bits!(
                            $mode, $min, $max, $max_error
                        ) as u32)
                        .unwrap_or(0)
                        .wrapping_sub(1)
                }
            }
        }
    };
}

#[macro_export]
/// threshold_slope is in unit / second
macro_rules! fixed_point_factory_slope {
    ($name:ident, $threshold_slope:literal, $sample_time_ms:literal, $max_error:literal) => {
        crate::fixed_point_factory!(
            $name,
            slope,
            f32,
            libm::roundf,
            $threshold_slope,
            $sample_time_ms,
            $max_error
        );
    };
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_fixed_point_factory_one_bit() {
        fixed_point_factory!(Factory, f32, 0.0, 1.0, 0.5);
        assert_eq!(Factory::to_fixed_point(0.0), Some(0.into()));
        assert_eq!(Factory::to_fixed_point(0.25), Some(0.into()));
        assert_eq!(Factory::to_fixed_point(0.5), Some(1.into()));
        assert_eq!(Factory::to_fixed_point(0.75), Some(1.into()));
        assert_eq!(Factory::to_fixed_point(1.0), Some(1.into()));
        assert_eq!(Factory::to_fixed_point(-1.0), None);
        assert_eq!(Factory::to_fixed_point(2.0), None);

        assert_relative_eq!(Factory::to_float(0.into()), 0.0, epsilon = 0.0001);
        assert_relative_eq!(Factory::to_float(1.into()), 1.0, epsilon = 0.0001);
    }

    #[test]
    fn test_fixed_point_factory_slope() {
        fixed_point_factory_slope!(Factory, 1.0, 1000.0, 1.0);
        assert_eq!(Factory::to_fixed_point(-1.0), Some(0.into()));
        assert_eq!(Factory::to_fixed_point(0.0), Some(1.into()));
        assert_eq!(Factory::to_fixed_point(1.0), Some(1.into()));

        assert_relative_eq!(Factory::to_float(0.into()), -1.0, epsilon = 0.0001);
        assert_relative_eq!(Factory::to_float(1.into()), 1.0, epsilon = 0.0001);
    }
}
