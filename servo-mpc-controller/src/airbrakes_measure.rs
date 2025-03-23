use std::{
    cmp::{max, min},
    f32::consts::PI,
    fs::File,
    time::Duration,
};

use csv::Writer;
use dspower_servo::DSPowerServo;
use log::{info, LevelFilter};
use serial::{MockServo, SerialWrapper};
use tokio::time::{self, Interval};
use tokio_serial::SerialPortBuilderExt as _;

mod serial;

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
            "tracking_angle",
            "actual_angle",
            "angular_velocity",
            "current",
            "pwm_duty_cycle",
            "temperature",
        ])
        .unwrap();

    #[cfg(not(feature = "mock_servo"))]
    let mut interval = time::interval(Duration::from_millis(10));
    #[cfg(feature = "mock_servo")]
    let mut interval = time::interval(Duration::from_micros(1));

    #[cfg(not(feature = "mock_servo"))]
    let (mut servo, angle) = {
        let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
            .open_native_async()
            .unwrap();
        let mut servo = DSPowerServo::new(SerialWrapper(serial));
        servo.init(true).await.unwrap();

        info!("Homing.....");
        servo.reduce_torque().await.unwrap();
        let mut angle = servo.batch_read_measurements().await.unwrap().angle;
        loop {
            angle -= 0.1;
            servo.move_to(angle).await.unwrap();
            interval.tick().await;
            let actual_angle = servo.batch_read_measurements().await.unwrap().angle;

            if angle < actual_angle - 5.0 {
                angle = actual_angle;
                servo.move_to(angle).await.unwrap();
                servo.restore_torque().await.unwrap();
                break;
            }
        }
        info!("Homing complete, angle at closed position: {}deg", angle);
        (servo, angle)
    };

    #[cfg(feature = "mock_servo")]
    let (mut servo, angle) = (MockServo::new(), -23.33);

    let amplitude = 46.66 / 2.0;
    let min_angle = angle;
    let max_angle = min_angle + amplitude * 2.0;
    let mut i = 0u32;

    info!("Starting measurements.....");

    // Measure step responses
    step(
        min_angle,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        max_angle,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        min_angle,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        min_angle + amplitude,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        max_angle,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        min_angle + amplitude,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        min_angle,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    step(
        min_angle + amplitude,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;

    // sine amplitude sweep
    let starting_params = SineParams::new(min_angle, max_angle);
    sine_amplitude_sweep(
        starting_params,
        2.0,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    sine_amplitude_sweep(
        starting_params,
        1.0,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    sine_amplitude_sweep(
        starting_params,
        0.75,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    sine_amplitude_sweep(
        starting_params,
        0.5,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    sine_amplitude_sweep(
        starting_params,
        0.35,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    sine_amplitude_sweep(
        starting_params,
        0.2,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;

    step(
        min_angle + amplitude,
        &mut servo,
        &mut interval,
        &mut i,
        &mut csv_writer,
    )
    .await;
    info!("Measurements complete");
    csv_writer.flush().unwrap();
}

async fn step(
    angle: f32,
    #[cfg(not(feature = "mock_servo"))] servo: &mut DSPowerServo<SerialWrapper>,
    #[cfg(feature = "mock_servo")] servo: &mut MockServo,
    interval: &mut Interval,
    i: &mut u32,
    writer: &mut Writer<File>,
) {
    for _ in 0..100 {
        let measurements = servo.batch_read_measurements().await.unwrap();
        servo.move_to(angle).await.unwrap();
        let t = *i as f32 * 0.01;
        writer
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

        *i += 1;
        interval.tick().await;
    }
}

#[derive(Debug, Clone, Copy)]
struct SineParams {
    min: f32,
    max: f32,
}

impl SineParams {
    fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }

    fn mid(&self) -> f32 {
        (self.min + self.max) / 2.0
    }

    fn amplitude(&self) -> f32 {
        (self.max - self.min) / 2.0
    }

    fn set_amplitude(&mut self, amplitude: f32) {
        let mid = self.mid();
        self.min = mid - amplitude;
        self.max = mid + amplitude;
    }

    fn set_mid(&mut self, mid: f32) {
        let amplitude = self.amplitude();
        self.min = mid - amplitude;
        self.max = mid + amplitude;
    }
}

async fn sine(
    params: SineParams,
    period_s: f32,
    #[cfg(not(feature = "mock_servo"))] servo: &mut DSPowerServo<SerialWrapper>,
    #[cfg(feature = "mock_servo")] servo: &mut MockServo,
    interval: &mut Interval,
    i: &mut u32,
    writer: &mut Writer<File>,
) -> SineParams {
    let ticks = (period_s / 0.01) as u32;
    let period_count = max(200 / ticks, 3);

    let mut min = f32::MAX;
    let mut max = f32::MIN;
    for tick in 0..(ticks * period_count) {
        let angle =
            ((tick as f32) * PI * 2.0 / (ticks as f32)).sin() * params.amplitude() + params.mid();
        let measurements = servo.batch_read_measurements().await.unwrap();
        servo.move_to(angle).await.unwrap();

        if tick > ticks {
            if measurements.angle < min {
                min = measurements.angle;
            }
            if measurements.angle > max {
                max = measurements.angle;
            }
        }

        let t = *i as f32 * 0.01;
        writer
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

        *i += 1;
        interval.tick().await;
    }

    SineParams::new(min, max)
}

async fn sine_amplitude_sweep(
    starting_params: SineParams,
    period_s: f32,
    #[cfg(not(feature = "mock_servo"))] servo: &mut DSPowerServo<SerialWrapper>,
    #[cfg(feature = "mock_servo")] servo: &mut MockServo,
    interval: &mut Interval,
    i: &mut u32,
    writer: &mut Writer<File>,
) {
    let mut params = starting_params;
    loop {
        let sine_result = sine(params, period_s, servo, interval, i, writer).await;
        // if sine_result.amplitude() > starting_params.amplitude() * 0.95 {
        //     break;
        // }
        let gain = (starting_params.amplitude() / sine_result.amplitude() - 1.0) * 0.3;

        if gain < 0.03 {
            break;
        }
        params.set_amplitude(params.amplitude() * (1.0 + gain));
        params.set_mid(sine_result.mid());

        info!("New params: {:?} amplitude: {}", params, params.amplitude());

        if params.min < -120.0 {
            let diff = -120.0 - params.min;
            params.min += diff;
            params.max -= diff;

            sine(params, period_s, servo, interval, i, writer).await;
            break;
        }
        if params.max > 120.0 {
            let diff = params.max - 120.0;
            params.min += diff;
            params.max -= diff;

            sine(params, period_s, servo, interval, i, writer).await;
            break;
        }
    }
}
