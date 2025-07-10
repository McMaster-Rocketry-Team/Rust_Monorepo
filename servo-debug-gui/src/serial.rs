use dspower_servo::Measurements;
use embedded_io_async::{ErrorType, Read, Write};
use nalgebra::{Matrix1x2, Matrix2, Matrix2x1};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[derive(Debug)]
pub struct SerialWrapper(pub tokio_serial::SerialStream);

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

pub struct MockServo {
    A: Matrix2<f32>,
    B: Matrix2x1<f32>,
    C: Matrix1x2<f32>,
    x: Matrix2x1<f32>,
}

impl MockServo {
  pub fn new() -> Self {
        let A = Matrix2::new(0.9394f32, 0.06012, -0.2254, 0.7754);
        let B = Matrix2x1::new(-1.055e-5f32, 0.0003027);
        let C = Matrix1x2::new(1635.0f32, 39.75);
        let x = Matrix2x1::new(0.0f32, 0.0);

        Self { A, B, C, x }
    }

    pub async fn batch_read_measurements(&self) -> Result<Measurements, ()> {
        Ok(Measurements {
            angle: (self.C * self.x)[0],
            angular_velocity: 0,
            current: 0.0,
            pwm_duty_cycle: 0.0,
            temperature: 0,
        })
    }

    pub async fn move_to(&mut self, angle: f32) -> Result<(), ()> {
        self.x = self.A * self.x + self.B * angle;
        Ok(())
    }
}
