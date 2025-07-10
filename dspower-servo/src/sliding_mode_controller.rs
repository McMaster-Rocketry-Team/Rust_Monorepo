use core::ops::Range;

use embedded_io_async::{Read, Write};

use crate::{DSPowerServo, DSPowerServoError, Measurements};

pub struct ServoSlidingModeController<'a, S>
where
    S: Read + Write,
{
    servo: &'a mut DSPowerServo<S>,
    rotation_range_deg: Range<f32>,

    gain: f32,
    dead_zone_angle_deg: f32,
}

impl<'a, S> ServoSlidingModeController<'a, S>
where
    S: Read + Write,
{
    /// the caller is responsible for initializing and homing the servo
    ///
    /// ### transition_angle_deg
    ///
    /// when the difference between commanded angle and actual angle is
    /// larger than this, full duty cycle is used
    ///
    /// ### dead_zone_angle_deg
    ///
    /// when the difference between commanded angle and actual angle is
    /// smaller than this, duty cycle is set to 0
    ///
    /// ---
    ///
    /// Error vs duty cycle graph looks like this:
    ///
    /// ```
    /// ──────────
    ///           ╲
    ///            │
    ///            └─┐
    ///              │
    ///               ╲
    ///                ──────────
    /// ```
    ///
    pub fn new(
        servo: &'a mut DSPowerServo<S>,
        rotation_range_deg: Range<f32>,
        gain: f32,
        dead_zone_angle_deg: f32,
    ) -> Self {
        Self {
            servo,
            rotation_range_deg,
            gain,
            dead_zone_angle_deg,
        }
    }

    /// need to be called at 100Hz
    pub async fn step(&mut self, command_angle: f32) -> Result<Measurements, DSPowerServoError<S>> {
        let m = self.servo.batch_read_measurements().await?;

        let error = command_angle - m.angle;

        if error.abs() < self.dead_zone_angle_deg {
            self.servo.move_to(command_angle).await?;
        } else {
            let mut new_command_angle = m.angle + error * self.gain;
            if new_command_angle > self.rotation_range_deg.end {
                new_command_angle = self.rotation_range_deg.end;
            } else if new_command_angle < self.rotation_range_deg.start {
                new_command_angle = self.rotation_range_deg.start;
            }

            self.servo.move_to(new_command_angle).await?;
        }

        Ok(m)
    }
}
