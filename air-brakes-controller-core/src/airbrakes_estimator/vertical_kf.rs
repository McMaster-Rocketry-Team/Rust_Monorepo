use nalgebra::{Matrix2, Vector2};

/// The v2 vertical channel: a plain linear 2-state Kalman filter
/// [altitude ASL, vertical velocity], constructed only once the baro is
/// trusted ("born subsonic" — see the plan doc). Predicts with the
/// earth-frame vertical acceleration from the dead reckoner attitude,
/// corrects with port-corrected baro altitude. Linear, 2x2, no Jacobians,
/// no cross-covariance path into attitude — that whole bug class from the
/// v1 EKF cannot be expressed here.
///
/// Every baro sample is fused; there is no innovation gate. One stood here
/// until 2026-08-18, rejecting altitudes more than 100 m from the prediction
/// and re-anchoring after 2 s of continuous disagreement. What it was built
/// for was an ejection-blast transient or a shock-disturbed static port, and
/// this filter cannot meet either: it is born subsonic and after burnout, and
/// it is retired at apogee, so its whole life is the one window of the flight
/// with no shock front ahead of the ports and no charge fired behind them.
/// A gate that cannot fire is not free — it is a threshold, a streak clock and
/// a recovery path that no flight exercises, in the code the brakes fly on.
/// The deployment estimator keeps its gate: that one flies through both.
///
/// All steps take measured dt; nothing assumes a sample rate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct VerticalKF {
    /// [altitude ASL (m), vertical velocity (m/s, up positive)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    x: Vector2<f32>,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    p: Matrix2<f32>,
    /// Process noise: std-dev of the acceleration input error (m/s^2) —
    /// attitude misprojection + accel bias/scale residue.
    q_accel_std: f32,
    /// Baro altitude measurement noise std-dev (m), after port correction.
    r_alt_std: f32,
}

impl VerticalKF {
    /// Construct at birth: altitude from a median of fresh port-corrected
    /// baro readings, velocity from the dead reckoner, with the given
    /// initial velocity uncertainty (larger on a forced / T_max birth).
    pub fn born(
        altitude_asl: f32,
        vertical_velocity: f32,
        velocity_std: f32,
        q_accel_std: f32,
        r_alt_std: f32,
    ) -> Self {
        Self {
            x: Vector2::new(altitude_asl, vertical_velocity),
            p: Matrix2::new(r_alt_std * r_alt_std, 0.0, 0.0, velocity_std * velocity_std),
            q_accel_std,
            r_alt_std,
        }
    }

    pub fn altitude_asl(&self) -> f32 {
        self.x[0]
    }

    pub fn vertical_velocity(&self) -> f32 {
        self.x[1]
    }

    /// Predict over measured `dt` with the earth-frame vertical linear
    /// acceleration (gravity already removed by the dead reckoner).
    pub fn predict(&mut self, accel_up: f32, dt: f32) {
        self.x[0] += self.x[1] * dt + 0.5 * accel_up * dt * dt;
        self.x[1] += accel_up * dt;

        // F = [[1, dt], [0, 1]]; Q from white acceleration noise
        let (p00, p01, p10, p11) = (self.p[(0, 0)], self.p[(0, 1)], self.p[(1, 0)], self.p[(1, 1)]);
        let q = self.q_accel_std * self.q_accel_std;
        self.p[(0, 0)] = p00 + dt * (p01 + p10) + dt * dt * p11 + q * dt * dt * dt * dt / 4.0;
        self.p[(0, 1)] = p01 + dt * p11 + q * dt * dt * dt / 2.0;
        self.p[(1, 0)] = p10 + dt * p11 + q * dt * dt * dt / 2.0;
        self.p[(1, 1)] = p11 + q * dt * dt;
    }

    /// Fuse one port-corrected baro altitude. Unconditionally: see the type
    /// doc for why this filter has no gate to refuse one.
    pub fn update(&mut self, corrected_alt_asl: f32) {
        let innovation = corrected_alt_asl - self.x[0];

        let r = self.r_alt_std * self.r_alt_std;
        let s = self.p[(0, 0)] + r;
        let k0 = self.p[(0, 0)] / s;
        let k1 = self.p[(1, 0)] / s;

        self.x[0] += k0 * innovation;
        self.x[1] += k1 * innovation;

        // Joseph form for the 2x2, H = [1, 0]
        let (p00, p01, p10, p11) = (self.p[(0, 0)], self.p[(0, 1)], self.p[(1, 0)], self.p[(1, 1)]);
        let a = 1.0 - k0;
        self.p[(0, 0)] = a * a * p00 + k0 * k0 * r;
        self.p[(0, 1)] = a * (p01 - k1 * p00) + k0 * k1 * r;
        self.p[(1, 0)] = a * (p10 - k1 * p00) + k0 * k1 * r;
        self.p[(1, 1)] = p11 - k1 * (p01 + p10) + k1 * k1 * p00 + k1 * k1 * r;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Constant deceleration, perfect accel input, noiseless baro: the
    /// filter must track both states closely.
    #[test]
    fn tracks_constant_deceleration() {
        let mut kf = VerticalKF::born(1000.0, 200.0, 5.0, 0.5, 3.0);
        let dt = 1.0 / 416.0;
        let accel = -15.0f32; // drag + gravity, m/s^2 (linear accel, up +)
        let mut true_alt_asl = 1000.0f32;
        let mut true_vv = 200.0f32;
        for _ in 0..(416 * 5) {
            true_alt_asl += true_vv * dt + 0.5 * accel * dt * dt;
            true_vv += accel * dt;
            kf.predict(accel, dt);
            kf.update(true_alt_asl);
        }
        assert!((kf.altitude_asl() - true_alt_asl).abs() < 1.0);
        assert!((kf.vertical_velocity() - true_vv).abs() < 0.5);
    }

    /// A wrong velocity at birth must be pulled back by the baro within a
    /// couple of seconds (this is what the birth velocity variance is for).
    #[test]
    fn recovers_from_wrong_birth_velocity() {
        let mut kf = VerticalKF::born(1000.0, 250.0, 30.0, 0.5, 3.0);
        let dt = 1.0 / 416.0;
        let accel = -15.0f32;
        let mut true_alt_asl = 1000.0f32;
        let mut true_vv = 200.0f32; // filter thinks 250 — 50 m/s wrong
        for _ in 0..(416 * 2) {
            true_alt_asl += true_vv * dt + 0.5 * accel * dt * dt;
            true_vv += accel * dt;
            kf.predict(accel, dt);
            kf.update(true_alt_asl);
        }
        assert!(
            (kf.vertical_velocity() - true_vv).abs() < 10.0,
            "vv err {} after 2 s",
            kf.vertical_velocity() - true_vv
        );
    }

    /// A baro spike now goes straight into the filter, because nothing can
    /// produce one where this filter flies. What the old gate bought — and
    /// what is being given up — is bounded by this: one 500 m garbage sample
    /// moves altitude by the Kalman gain, not by 500 m.
    #[test]
    fn a_spike_is_fused_not_rejected() {
        let mut kf = VerticalKF::born(1000.0, 100.0, 5.0, 0.5, 3.0);
        let dt = 1.0 / 416.0;
        kf.predict(-15.0, dt);
        let before = kf.altitude_asl();
        kf.update(before + 500.0);
        let jump = kf.altitude_asl() - before;
        // p00 at birth is r^2, so the gain is 1/2 and the filter takes half
        // the spike. That is the cost of no gate on a single bad sample, and
        // it is why the deployment estimator — which flies through ejection
        // charges — keeps one.
        assert!(
            jump > 200.0 && jump < 300.0,
            "a 500 m spike moved altitude {jump} m"
        );
    }
}
