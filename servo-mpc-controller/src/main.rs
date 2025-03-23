#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use std::{array, cmp::min, f32::consts::PI, time::Duration};

use csv::Writer;
use dspower_servo::{DSPowerServo, Measurements};
use embedded_io_async::{ErrorType, Read, Write};
use log::{info, LevelFilter};
use nalgebra::{Matrix1, Matrix1x2, Matrix2, Matrix2x1, SMatrix};
use osqp::{CscMatrix, Problem};
use serial::MockServo;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time,
};
use tokio_serial::SerialPortBuilderExt;

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
            "commanded_angle",
            "actual_angle",
            "angular_velocity",
            "current",
            "pwm_duty_cycle",
            "temperature",
        ])
        .unwrap();

    // let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //     .open_native_async()
    //     .unwrap();
    // let mut servo = DSPowerServo::new(SerialWrapper(serial));
    // servo.init(true).await.unwrap();
    let mut servo = MockServo::new();

    // ======================== State space model of the servo
    // https://www.notion.so/mcmasterrocketry/Servo-Simulink-Model-51e64ae3558446ad83d20f000c16cc0b
    let A = Matrix2::new(0.9394f32, 0.06012, -0.2254, 0.7754);
    let At = A.transpose();
    let B = Matrix2x1::new(-1.055e-5f32, 0.0003027);
    let C = Matrix1x2::new(1635.0f32, 39.75);
    let Ct = C.transpose();

    // ======================== Values needed for Kalman filter
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

    // ======================== Values needed for MPC
    // Prediction horizon
    const Np: usize = 20;
    // Control horizon
    const Nc: usize = 10;

    let mut F = SMatrix::<f32, Np, 2>::zeros();
    let mut Phi = SMatrix::<f32, Np, Np>::zeros();
    let mut A_power = Matrix2::identity();
    for i in 0..Np {
        let value = C * A_power * B;
        let value = value[0];
        for j in 0..(Np - i) {
            Phi[(i + j, j)] = value;
        }

        A_power = A_power * A;

        let row = C * A_power;
        F.row_mut(i).copy_from(&row);
    }

    let w_q = 1.0; // Output tracking weight
    let w_r = 0.005; // Control effort weight

    let mut Q_bar = SMatrix::<f32, Np, Np>::zeros();
    for i in 0..Np {
        Q_bar[(i, i)] = w_q;
    }

    let mut R_bar = SMatrix::<f32, Nc, Nc>::zeros();
    for i in 0..Nc {
        R_bar[(i, i)] = w_r;
    }

    let mut T = SMatrix::<f32, Np, Nc>::zeros();
    for i in 0..Np {
        let j = min(i, Nc - 1);
        T[(i, j)] = 1.0;
    }

    let Phi = Phi * T;
    let Phi_t = Phi.transpose();

    // Define problem for OSQP
    let P_osqp = (Phi_t * Q_bar * Phi + R_bar) * 2.0;
    let P_osqp: [[f64; Nc]; Nc] = array::from_fn(|i| array::from_fn(|j| P_osqp[(i, j)] as f64));
    let P_osqp = CscMatrix::from(&P_osqp).into_upper_tri();
    let A_osqp = SMatrix::<f32, Nc, Nc>::identity();
    let A_osqp: [[f64; Nc]; Nc] = array::from_fn(|i| array::from_fn(|j| A_osqp[(i, j)] as f64));
    let l_osqp = [-120.0f64; Nc];
    let u_osqp = [120.0f64; Nc];

    let osqp_settings = osqp::Settings::default().verbose(false);

    // ======================== Main loop
    let angle_commands = create_angle_commands();
    let mut interval = time::interval(Duration::from_millis(10));
    for (i, tracking_angle) in angle_commands.iter().enumerate() {
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
        let r_bar = SMatrix::<f32, Np, 1>::from_element(*tracking_angle);
        let q_osqp = (Phi_t * Q_bar * (F * x - r_bar)) * 2.0;
        let q_osqp: [f64; Nc] = array::from_fn(|i| q_osqp[i] as f64);
        let mut qp =
            Problem::new(&P_osqp, &q_osqp, &A_osqp, &l_osqp, &u_osqp, &osqp_settings).unwrap();
        let result = qp.solve();
        let angle = result.x().unwrap()[0] as f32;
        // let angle = *tracking_angle;

        // interval.tick().await;

        let measurements = servo.batch_read_measurements().await.unwrap();
        servo.move_to(angle).await.unwrap();

        z = Matrix1::new(measurements.angle);
        u = Some(Matrix1::new(angle));

        let t = i as f32 * 0.01 - 1.0;
        if t >= 0.0 {
            csv_writer
                .write_record(&[
                    t.to_string(),
                    (*tracking_angle).to_string(),
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
    for amplitude in [30
    // , 30, 50, 70
    ].iter() {
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
