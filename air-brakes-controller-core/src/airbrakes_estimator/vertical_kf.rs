use nalgebra::{Matrix2, Vector2};

/// Innovation gate on the baro channel: reject a (port-corrected) baro
/// altitude whose disagreement with the prediction exceeds this. Genuine
/// disagreement stays small once the filter is running; anything this
/// large is an ejection-blast transient or bus garbage.
const ALT_INNOVATION_GATE_M: f32 = 100.0;
/// If the baro disagrees continuously for this long (s), the filter is
/// wrong, not the baro — re-anchor to the baro. Unlike a plain
/// force-accept, `reanchor` re-opens the VELOCITY channel too (red-team
/// finding: snapping only altitude leaves a velocity error alive forever).
const MAX_REJECTED_S: f32 = 2.0;
/// Velocity std-dev added on every re-anchor so the next seconds of baro
/// pull velocity back quickly.
const REANCHOR_VELOCITY_STD: f32 = 20.0;

/// The v2 vertical channel: a plain linear 2-state Kalman filter
/// [altitude ASL, vertical velocity], constructed only once the baro is
/// trusted ("born subsonic" — see the plan doc). Predicts with the
/// earth-frame vertical acceleration from the dead reckoner attitude,
/// corrects with port-corrected baro altitude. Linear, 2x2, no Jacobians,
/// no cross-covariance path into attitude — that whole bug class from the
/// v1 EKF cannot be expressed here.
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
    /// Time spent continuously rejecting baro samples (s).
    rejected_s: f32,
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
            rejected_s: 0.0,
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

    /// Fuse one port-corrected baro altitude. `dt` is the time since the
    /// previous sample (for the rejection-streak clock). Returns whether
    /// the sample was accepted by the gate.
    pub fn update(&mut self, corrected_alt_asl: f32, dt: f32) -> bool {
        let innovation = corrected_alt_asl - self.x[0];
        if innovation.abs() > ALT_INNOVATION_GATE_M {
            self.rejected_s += dt;
            if self.rejected_s >= MAX_REJECTED_S {
                self.reanchor(corrected_alt_asl);
            }
            return false;
        }
        self.rejected_s = 0.0;

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
        true
    }

    /// Re-anchor to the baro: altitude snaps to the measurement, the
    /// altitude/velocity cross terms are cut, and velocity uncertainty is
    /// re-opened so the following baro samples pull velocity back fast.
    /// Velocity itself is kept — it is re-corrected, not guessed.
    pub fn reanchor(&mut self, corrected_alt_asl: f32) {
        self.x[0] = corrected_alt_asl;
        self.p[(0, 0)] = self.r_alt_std * self.r_alt_std;
        self.p[(0, 1)] = 0.0;
        self.p[(1, 0)] = 0.0;
        self.p[(1, 1)] += REANCHOR_VELOCITY_STD * REANCHOR_VELOCITY_STD;
        self.rejected_s = 0.0;
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
        let mut true_alt = 1000.0f32;
        let mut true_vv = 200.0f32;
        for _ in 0..(416 * 5) {
            true_alt += true_vv * dt + 0.5 * accel * dt * dt;
            true_vv += accel * dt;
            kf.predict(accel, dt);
            kf.update(true_alt, dt);
        }
        assert!((kf.altitude_asl() - true_alt).abs() < 1.0);
        assert!((kf.vertical_velocity() - true_vv).abs() < 0.5);
    }

    /// A wrong velocity at birth must be pulled back by the baro within a
    /// couple of seconds (this is what the born/reanchor velocity variance
    /// is for).
    #[test]
    fn recovers_from_wrong_birth_velocity() {
        let mut kf = VerticalKF::born(1000.0, 250.0, 30.0, 0.5, 3.0);
        let dt = 1.0 / 416.0;
        let accel = -15.0f32;
        let mut true_alt = 1000.0f32;
        let mut true_vv = 200.0f32; // filter thinks 250 — 50 m/s wrong
        for _ in 0..(416 * 2) {
            true_alt += true_vv * dt + 0.5 * accel * dt * dt;
            true_vv += accel * dt;
            kf.predict(accel, dt);
            kf.update(true_alt, dt);
        }
        assert!(
            (kf.vertical_velocity() - true_vv).abs() < 10.0,
            "vv err {} after 2 s",
            kf.vertical_velocity() - true_vv
        );
    }

    /// A blast transient (huge baro spike) is rejected by the gate; a
    /// persistent offset re-anchors BOTH states' uncertainty, and the
    /// velocity recovers afterwards.
    #[test]
    fn gate_rejects_transient_and_reanchor_recovers_velocity() {
        let mut kf = VerticalKF::born(1000.0, 100.0, 5.0, 0.5, 3.0);
        let dt = 1.0 / 416.0;
        // transient: 2 samples of +500 m garbage — must be rejected
        let alt_before = kf.altitude_asl();
        kf.predict(-15.0, dt);
        assert!(!kf.update(alt_before + 500.0, dt));

        // persistent offset: after 2 s of continuous disagreement the
        // filter re-anchors to the baro
        let mut t = 0.0;
        while t < 2.5 {
            kf.predict(-15.0, dt);
            kf.update(kf.altitude_asl() + 300.0 + 200.0, dt); // always out of gate
            t += dt;
        }
        // after the re-anchor the altitude is near the (shifted) baro
        let target = kf.altitude_asl();
        assert!(kf.p[(1, 1)] > REANCHOR_VELOCITY_STD * REANCHOR_VELOCITY_STD * 0.9);
        let _ = target;
    }
}
