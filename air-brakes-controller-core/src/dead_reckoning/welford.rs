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
