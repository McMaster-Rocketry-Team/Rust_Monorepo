use nalgebra::{SMatrix, SVector};

/// Compute the Jacobian J ∈ R^{M×N} of `f` at point `x` via central differences.
/// - `f`: maps R^N → R^M, must be pure/deterministic around `x`.
/// - `x`: evaluation point.
/// Returns: J where J[i,j] = ∂f_i/∂x_j evaluated at `x`.
pub fn central_difference_jacobian<const M: usize, const N: usize, F>(
    x: &SVector<f32, N>,
    f: F,
) -> SMatrix<f32, M, N>
where
    F: Fn(&SVector<f32, N>) -> SVector<f32, M>,
{
    // (f32::EPSILON)^(1/3) ≈ 0.0049215666; optimal scaling for central difference.
    const EPS_CBRT: f32 = 0.004_921_566_6;

    let mut j = SMatrix::<f32, M, N>::zeros();

    // Reuse vectors on the stack to avoid any allocation.
    let mut xp = *x;
    let mut xm = *x;

    let mut col = 0usize;
    while col < N {
        let xi = x[col];
        // Scale step with magnitude of xi to balance absolute/relative errors.
        let h = EPS_CBRT * if xi.is_finite() { xi.abs().max(1.0) } else { 1.0 };

        xp[col] = xi + h;
        xm[col] = xi - h;

        let fp = f(&xp);
        let fm = f(&xm);

        let inv_2h = 0.5f32 / h;
        let mut row = 0usize;
        while row < M {
            j[(row, col)] = (fp[row] - fm[row]) * inv_2h;
            row += 1;
        }

        // restore and advance
        xp[col] = xi;
        xm[col] = xi;
        col += 1;
    }

    j
}
