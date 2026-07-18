use core::ops::Range;

use embedded_io_async::{Read, Write};

use crate::{HiwonderServo, HiwonderServoError};

pub struct ServoSlidingModeController<'a, 'b, S>
where
    S: Read + Write,
{
    servo: &'a mut HiwonderServo<'b, S>,
    rotation_range_deg: Range<f32>,

    gain: f32,
    dead_zone_angle_deg: f32,
}

impl<'a, 'b, S> ServoSlidingModeController<'a, 'b, S>
where
    S: Read + Write,
{
    /// the caller is responsible for initializing and homing the servo
    ///
    /// ### gain
    ///
    /// fraction of the remaining error the controller commands each step; a
    /// value below 1 softens the approach so the servo does not overshoot
    ///
    /// ### dead_zone_angle_deg
    ///
    /// when the difference between commanded angle and actual angle is
    /// smaller than this, the target is commanded directly
    ///
    /// ---
    ///
    /// Error vs commanded step graph looks like this:
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
        servo: &'a mut HiwonderServo<'b, S>,
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
    ///
    /// Returns the measured angle (degrees) sampled at the start of the step.
    pub async fn step(&mut self, command_angle: f32) -> Result<f32, HiwonderServoError<S>> {
        let measured_angle = self.servo.read_position().await?;

        let error = command_angle - measured_angle;

        // move_to always drives at full speed toward each intermediate target;
        // the gain term is what tames the approach.
        if error.abs() < self.dead_zone_angle_deg {
            self.servo.move_to(command_angle).await?;
        } else {
            let mut new_command_angle = measured_angle + error * self.gain;
            if new_command_angle > self.rotation_range_deg.end {
                new_command_angle = self.rotation_range_deg.end;
            } else if new_command_angle < self.rotation_range_deg.start {
                new_command_angle = self.rotation_range_deg.start;
            }

            self.servo.move_to(new_command_angle).await?;
        }

        Ok(measured_angle)
    }
}
