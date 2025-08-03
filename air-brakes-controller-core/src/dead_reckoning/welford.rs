/// Online estimator for mean and variance of 3-component f32 samples.
#[derive(Debug, Clone)]
pub struct Welford3 {
    count: u32,
    mean: [f32; 3],
    m2:   [f32; 3],
}

impl Welford3 {
    /// Create a new, empty estimator.
    pub fn new() -> Self {
        Self {
            count: 0,
            mean:  [0.0; 3],
            m2:    [0.0; 3],
        }
    }

    /// Incorporate one new 3-vector sample.
    pub fn update(&mut self, x: &[f32; 3]) {
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
    pub fn mean(&self) -> [f32; 3] {
        self.mean
    }

    /// Population variance (σ²). Returns None until at least one sample.
    pub fn variance(&self) -> Option<[f32; 3]> {
        if self.count > 0 {
            let n = self.count as f32;
            Some([
                self.m2[0] / n,
                self.m2[1] / n,
                self.m2[2] / n,
            ])
        } else {
            None
        }
    }

    /// Sample variance (unbiased, uses n–1). Returns None until ≥2 samples.
    pub fn sample_variance(&self) -> Option<[f32; 3]> {
        if self.count > 1 {
            let n = (self.count as f32) - 1.0;
            Some([
                self.m2[0] / n,
                self.m2[1] / n,
                self.m2[2] / n,
            ])
        } else {
            None
        }
    }
}
