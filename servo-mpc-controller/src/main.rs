use std::{array, f32::consts::PI, time::Duration};

use csv::Writer;
use dspower_servo::DSPowerServo;
use embedded_io_async::{ErrorType, Read, Write};
use log::{info, LevelFilter};
use nalgebra::{Matrix1, Matrix1x2, Matrix2, Matrix2x1, Matrix3x4};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time,
};
use tokio_serial::SerialPortBuilderExt;

#[derive(Debug)]
struct SerialWrapper(tokio_serial::SerialStream);

impl ErrorType for SerialWrapper {
    type Error = std::io::Error;
}

impl Read for SerialWrapper {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf).await
    }
}

impl Write for SerialWrapper {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).await
    }
}

#[tokio::main]
async fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .try_init()
        .unwrap();

    let mut csv_writer = Writer::from_path("output.csv").unwrap();
    csv_writer
        .write_record(&[
            "timestamp",
            "commanded_angle",
            "actual_angle",
            "angular_velocity",
            "current",
            "pwm_duty_cycle",
            "temperature",
        ])
        .unwrap();

    let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
        .open_native_async()
        .unwrap();
    let mut servo = DSPowerServo::new(SerialWrapper(serial));
    servo.init(true).await.unwrap();

    // State space model of the servo
    // https://www.notion.so/mcmasterrocketry/Servo-Simulink-Model-51e64ae3558446ad83d20f000c16cc0b
    let A = Matrix2::new(0.9394f32, 0.06012, -0.2254, 0.7754);
    let At = A.transpose();
    let B = Matrix2x1::new(-1.055e-5f32, 0.0003027);
    let C = Matrix1x2::new(1635.0f32, 39.75);
    let Ct = C.transpose();

    // Sensor noise (variance)
    let r = Matrix1::new(0.01f32);

    // Process noise (variance)
    let q = Matrix2::new(0.05f32, 0.0, 0.0, 0.05f32);

    // Covariance
    let mut P = Matrix2::new(10000.0f32, 0.0, 0.0, 10000.0);
    // State estimate
    let mut x = Matrix2x1::new(0.0f32, 0.0);
    // Last sensor measurement
    let mut z = Matrix1::new(servo.batch_read_measurements().await.unwrap().angle);
    // Last input
    let mut u: Option<Matrix1<f32>> = None;

    let angle_commands = create_angle_commands();
    let mut interval = time::interval(Duration::from_millis(10));
    for (i, angle) in angle_commands.iter().enumerate() {
        // Combine estimated state with sensor measurement
        let K = P * Ct * (C * P * Ct + r).try_inverse().unwrap();
        x = x + K * (z - C * x);
        P = P - K * C * P;

        if let Some(u) = u {
            // Predict next state
            x = A * x + B * u;
            P = A * P * At + q;
        }

        // Calculate next input
        let angle = *angle;

        interval.tick().await;

        let measurements = servo.batch_read_measurements().await.unwrap();
        servo.move_to(angle).await.unwrap();

        z = Matrix1::new(measurements.angle);
        u = Some(Matrix1::new(angle));

        let t = i as f32 * 0.01 - 1.0;
        if t >= 0.0 {
            csv_writer
                .write_record(&[
                    t.to_string(),
                    angle.to_string(),
                    measurements.angle.to_string(),
                    measurements.angular_velocity.to_string(),
                    measurements.current.to_string(),
                    measurements.pwm_duty_cycle.to_string(),
                    measurements.temperature.to_string(),
                ])
                .unwrap();
        }
    }
}

fn create_angle_commands() -> Vec<f32> {
    let mut angles = Vec::new();

    // reset to zero
    angles.append(&mut vec![0.0; 100]);

    // step inputs
    for angle in [10, 30, 50, 70, 90, 110, 130].iter() {
        angles.append(&mut vec![0.0; 100]);
        angles.append(&mut vec![*angle as f32; 100]);
    }

    // frequency sweeps
    for amplitude in [10, 30, 50, 70].iter() {
        angles.append(&mut vec![0.0; 100]);

        let mut t = 0.0f32;
        while t < 40.0 {
            let angle = (t * PI * (1.1f32.powf(t)) / 10.0).sin() * (*amplitude as f32 / 2.0);
            angles.push(angle);

            t += 0.01;
        }
    }

    angles.append(&mut vec![0.0; 100]);

    angles
}
