use nalgebra::SVector;


/// Online estimator for mean and variance of 3-component f32 samples.
#[derive(Debug, Clone)]
pub struct Welford<const N: usize> {
    count: u32,
    mean: SVector<f32, N>,
    m2:   SVector<f32, N>,
}

impl<const N: usize> Welford<N> {
    /// Create a new, empty estimator.
    pub fn new() -> Self {
        Self {
            count: 0,
            mean:  SVector::zeros(),
            m2:    SVector::zeros(),
        }
    }

    pub fn update(&mut self, x: &SVector<f32, N>) {
        self.count += 1;
        let n = self.count as f32;

        for i in 0..x.len() {
            let delta = x[i] - self.mean[i];
            // update mean
            self.mean[i] += delta / n;
            // update sum of squares of differences
            let delta2 = x[i] - self.mean[i];
            self.m2[i] += delta * delta2;
        }
    }

    pub fn mean(&self) -> SVector<f32, N> {
        self.mean
    }

    pub fn variance(&self) -> Option<SVector<f32, N>> {
        if self.count > 0 {
            let n = self.count as f32;
            Some(self.m2 / n)
        } else {
            None
        }
    }

    pub fn variance_magnitude(&self) -> Option<f32> {
        self.variance().map(|v|v.magnitude())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    #[test]
    fn test_mean() {
        let mut welford = Welford::<3>::new();
        
        // Test with no samples
        assert_eq!(welford.mean(), Vector3::zeros());
        
        // Test with one sample
        let sample1 = Vector3::new(1.0, 2.0, 3.0);
        welford.update(&sample1);
        assert_relative_eq!(welford.mean(), sample1, epsilon = 1e-6);
        
        // Test with two samples
        let sample2 = Vector3::new(3.0, 4.0, 5.0);
        welford.update(&sample2);
        let expected_mean = Vector3::new(2.0, 3.0, 4.0); // (1+3)/2, (2+4)/2, (3+5)/2
        assert_relative_eq!(welford.mean(), expected_mean, epsilon = 1e-6);
        
        // Test with three samples
        let sample3 = Vector3::new(5.0, 6.0, 7.0);
        welford.update(&sample3);
        let expected_mean = Vector3::new(3.0, 4.0, 5.0); // (1+3+5)/3, (2+4+6)/3, (3+5+7)/3
        assert_relative_eq!(welford.mean(), expected_mean, epsilon = 1e-6);
        
        // Test with negative values
        let sample4 = Vector3::new(-2.0, -4.0, -6.0);
        welford.update(&sample4);
        let expected_mean = Vector3::new(1.75, 2.0, 2.25); // (1+3+5-2)/4, (2+4+6-4)/4, (3+5+7-6)/4
        assert_relative_eq!(welford.mean(), expected_mean, epsilon = 1e-6);
    }
}
