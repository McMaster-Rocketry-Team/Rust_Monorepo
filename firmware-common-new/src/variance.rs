#[allow(unused_imports)]
use micromath::F32Ext;

/// Online mean / variance calculator for an IMU stream.
/// Generic over the number of channels `N` (e.g. 3 for accel, 6 for accel+gyro).
///
/// Updating cost per sample is O(N) and uses Welford’s algorithm
/// to avoid catastrophic cancellation.
///
/// Example:
/// ```
/// let mut stats = RunningStats::<6>::new();
/// stats.update([ax, ay, az, gx, gy, gz]);
/// let var = stats.variance();
/// ```
pub struct VarianceEstimator<const N: usize> {
    n: u32,         // sample count
    mean: [f32; N], // running mean μ
    m2: [f32; N],   // Σ(x-μ)²  (used to derive variance)
}

impl<const N: usize> VarianceEstimator<N> {
    /// Create an empty accumulator.
    pub const fn new() -> Self {
        Self {
            n: 0,
            mean: [0.0; N],
            m2: [0.0; N],
        }
    }

    /// Add one multichannel sample.
    pub fn update(&mut self, sample: [f32; N]) {
        self.n = self.n.saturating_add(1);
        let n_f = self.n as f32;

        for i in 0..N {
            let delta = sample[i] - self.mean[i];
            self.mean[i] += delta / n_f; // μ ← μ + δ / n
            // `delta2` uses the updated mean
            let delta2 = sample[i] - self.mean[i];
            self.m2[i] += delta * delta2; // Σ(x-μ_old)(x-μ_new)
        }
    }

    /// Return per-channel mean.  Undefined for `n == 0`.
    pub fn mean(&self) -> [f32; N] {
        self.mean
    }

    /// Return per-channel **population** variance (σ²).
    /// For the unbiased *sample* variance use `self.m2/(n-1)` instead.
    pub fn variance(&self) -> [f32; N] {
        if self.n == 0 {
            [0.0; N]
        } else {
            let n_f = self.n as f32;
            core::array::from_fn(|i| self.m2[i] / n_f)
        }
    }

    /// If the input unit is deg/s, noise density have the unit of deg/s/√Hz
    pub fn noise_density(&self, sample_rate: f32) -> [f32; N] {
        let k = (2.0f32 / sample_rate).sqrt();
        let variance = self.variance();
        core::array::from_fn(|i| (variance[i].sqrt()) * k)
    }

    /// Reset the accumulator while keeping current capacity.
    pub fn clear(&mut self) {
        self.n = 0;
        self.mean = [0.0; N];
        self.m2 = [0.0; N];
    }
}

#[cfg(test)]
mod tests {
    use super::VarianceEstimator;
    use approx::assert_relative_eq;

    fn assert_close<const N: usize>(got: [f32; N], want: [f32; N]) {
        for i in 0..N {
            assert_relative_eq!(got[i], want[i], epsilon = 1e-6);
        }
    }

    #[test]
    fn zero_samples_is_zero() {
        let stats = VarianceEstimator::<3>::new();
        assert_eq!(stats.mean(), [0.0; 3]);
        assert_eq!(stats.variance(), [0.0; 3]);
    }

    #[test]
    fn one_sample_mean_matches() {
        let mut s = VarianceEstimator::<3>::new();
        s.update([1.0, -2.0, 3.5]);
        assert_eq!(s.mean(), [1.0, -2.0, 3.5]);
        // population variance of a single sample is zero
        assert_eq!(s.variance(), [0.0; 3]);
    }

    #[test]
    fn two_samples_known_result() {
        let mut s = VarianceEstimator::<1>::new();
        s.update([2.0]);
        s.update([4.0]);
        // mean = 3, population var = ((2-3)^2 + (4-3)^2)/2 = 1
        assert_close(s.mean(), [3.0]);
        assert_close(s.variance(), [1.0]);
    }

    #[test]
    fn many_samples_matches_hand_calc() {
        const DATA: &[[f32; 2]] = &[[1.0, 2.0], [2.0, 4.0], [3.0, 6.0], [4.0, 8.0]];
        let mut s = VarianceEstimator::<2>::new();
        for &d in DATA {
            s.update(d);
        }

        let mean_expected = [2.5, 5.0];
        let var_expected = [1.25, 5.0];

        assert_close(s.mean(), mean_expected);
        assert_close(s.variance(), var_expected);
    }

    #[test]
    fn clear_resets_state() {
        let mut s = VarianceEstimator::<1>::new();
        s.update([10.0]);
        s.clear();
        assert_eq!(s.mean(), [0.0]);
        assert_eq!(s.variance(), [0.0]);
        assert_eq!(s.n, 0);
    }
}
