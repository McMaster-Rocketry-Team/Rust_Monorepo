/// Air density (kg/m^3) at altitude ASL (m), ISA troposphere formula
/// (valid to 11 km).
///
/// The full formula rather than a low-altitude linear fit: LC'26 simulates
/// to 6+ km, and under-reading density over-predicts apogee, which
/// over-extends the airbrakes — a one-sided error in the harmful direction.
///
/// History worth keeping, because it is the reason this is a plain
/// polynomial and not a call to anything: this used to be written
/// `x.powf(4.256)` as a method. That form resolves to the inherent
/// `f32::powf` under std and to whatever `F32Ext` trait is in scope under
/// `no_std` — so the same source line computed different numbers on the host
/// and on the board. With `micromath` in scope it cost 39% of the density at
/// 10 km, inflated the Mach-lockout exit's drag inversion by 28%, and
/// delayed that exit by 3 s on the bench (measured, 2026-08-16). It was then
/// `libm::powf` called by name, which fixed the resolution but not the cost.
/// Evaluating the curve directly has neither problem: there is no name to
/// resolve, and no implementation whose accuracy can vary underneath it.
pub fn approximate_air_density(altitude_asl: f32) -> f32 {
    // Degree-4 least-squares fit (Chebyshev nodes) of that same curve in the
    // dimensionless u, replacing the `powf` call it used to make. On the M7
    // the `powf` measured 1207 cycles and was about a third of an airbrakes
    // MPC solve; this is ~10 flops with no branch, no table walk and no
    // transcendental. Max relative error against the exact f64 formula is
    // 1.6e-6 including f32 Horner rounding, measured on-target at 2.0e-6
    // worst (h = 10800 m) — f32 round-off, and four orders of magnitude
    // tighter than the CFD Cd it feeds. Apogee predictions across the flight
    // envelope are identical to the millimetre.
    //
    // `u` is clamped rather than the base being `.max(0.0)`: outside the fit
    // domain (h in [-1000, 12000] m) a polynomial diverges instead of
    // decaying, so density saturates at the edge value. That is not a loss of
    // fidelity — the ISA troposphere model this approximates is itself only
    // valid to 11 km — and it is strictly safer than the old behaviour, which
    // returned exactly 0.0 above 44.3 km and so could divide by zero in the
    // drag inversion in `airbrakes_estimator`.
    let u = (2.25577e-5 * altitude_asl).clamp(-0.03, 0.28);
    ((((1.936_849_5 * u - 6.368_716) * u + 8.486_722) * u - 5.213_590) * u) + 1.225_000_4
}

/// Square root that reaches the FPU.
///
/// `libm::sqrtf` dispatches to a hardware instruction only on aarch64/neon,
/// wasm32 and x86; on `thumbv7em-none-eabihf` it runs the generic software
/// algorithm — 714 cycles measured on the board, for something the FPv5 unit
/// does in one instruction. `normalize()` in the MPC's inner loop paid that
/// twice per RK2 step.
///
/// Unlike [`approximate_air_density`], swapping implementations here cannot
/// reintroduce a host/target divergence: IEEE-754 requires `sqrt` to be
/// correctly rounded, so every conforming implementation returns the same
/// bits. That is exactly the property `powf` lacks.
#[cfg(any(test, feature = "std"))]
#[inline(always)]
pub fn sqrt(x: f32) -> f32 {
    x.sqrt()
}

#[cfg(not(any(test, feature = "std")))]
#[inline(always)]
pub fn sqrt(x: f32) -> f32 {
    // Lowers to VSQRT.F32 on this target.
    core::intrinsics::sqrtf32(x)
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