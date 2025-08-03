use nalgebra::Vector3;

/// Online estimator for mean and variance of 3-component f32 samples.
#[derive(Debug, Clone)]
pub struct Welford3 {
    count: u32,
    mean: Vector3<f32>,
    m2:   Vector3<f32>,
}

impl Welford3 {
    /// Create a new, empty estimator.
    pub fn new() -> Self {
        Self {
            count: 0,
            mean:  Vector3::zeros(),
            m2:    Vector3::zeros(),
        }
    }

    /// Incorporate one new 3-vector sample.
    pub fn update(&mut self, x: &Vector3<f32>) {
        self.count += 1;
        let n = self.count as f32;

        for i in 0..3 {
            let delta = x[i] - self.mean[i];
            // update mean
            self.mean[i] += delta / n;
            // update sum of squares of differences
            let delta2 = x[i] - self.mean[i];
            self.m2[i] += delta * delta2;
        }
    }

    /// Current mean (population).
    pub fn mean(&self) -> Vector3<f32> {
        self.mean
    }

    /// Population variance (σ²). Returns None until at least one sample.
    pub fn variance(&self) -> Option<Vector3<f32>> {
        if self.count > 0 {
            let n = self.count as f32;
            Some(self.m2 / n)
        } else {
            None
        }
    }

    /// Sample variance (unbiased, uses n–1). Returns None until ≥2 samples.
    pub fn sample_variance(&self) -> Option<Vector3<f32>> {
        if self.count > 1 {
            let n = (self.count as f32) - 1.0;
            Some(self.m2 / n)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_mean() {
        let mut welford = Welford3::new();
        
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
