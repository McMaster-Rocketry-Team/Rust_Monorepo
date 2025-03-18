use std::{array, time::Duration};

use dspower_servo::DSPowerServo;
use embedded_io_async::{ErrorType, Read, Write};
use log::{info, LevelFilter};
use nalgebra::{DMatrix, DVector};
use osqp::{CscMatrix, Problem, Settings, Status};
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

    info!("Hello, world!");

    let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
        .open_native_async()
        .unwrap();
    let mut servo = DSPowerServo::new(SerialWrapper(serial));
    servo.init(true).await.unwrap();

    // State space model of the servo
    let A = DMatrix::from_row_slice(2, 2, &[0.9394, 0.06012, -0.2254, 0.7754]);
    let B = DMatrix::from_row_slice(2, 1, &[-1.055e-05, 0.00003027]);
    let C = DMatrix::from_row_slice(1, 2, &[1635.0, 39.75]);

    // MPC Parameters
    let n_x = A.nrows(); // 2

    // Prediction horizon (Np) and control horizon (Nc)
    let Np = 10;
    let Nc = 4;

    // Weights (scalars for SISO case)
    let q = 1.0; // Output tracking weight
    let r_u = 0.1; // Control effort weight

    // Create block-diagonal Q_bar (size Np x Np) and R_bar (size Np x Np)
    // For a single-output system, these become simple diagonals
    let mut Q_bar = DMatrix::from_element(Np, Np, 0.0);
    for i in 0..Np {
        Q_bar[(i, i)] = q;
    }

    let mut R_bar = DMatrix::from_element(Np, Np, 0.0);
    for i in 0..Np {
        R_bar[(i, i)] = r_u;
    }

    // -----------------------------------------
    // 3) Build prediction matrices F and Phi
    // -----------------------------------------
    // Y = F * x0 + Phi * U
    // where U = [u0, u1, ..., u_{Np-1}]^T
    //
    // F[i,:] = C * A^(i+1)
    // Phi[i,j] = C * A^(i-j) * B for j <= i
    let mut F = DMatrix::zeros(Np, n_x); // (Np x 2)
    let mut Phi = DMatrix::zeros(Np, Np); // (Np x Np) for SISO control

    // Build F
    let mut A_power = A.clone();
    for i in 0..Np {
        // A^(i+1)
        if i > 0 {
            A_power = A_power * A.clone();
        }
        // row i of F
        let row = C.clone() * A_power.clone();
        F.row_mut(i).copy_from(&row.row(0));
    }

    // Build Phi
    for i in 0..Np {
        let mut A_pow = DMatrix::identity(n_x, n_x);
        for j in 0..=i {
            // A^(i-j)
            if j > 0 {
                A_pow = A_pow * A.clone();
            }
            // Phi[i,j] = C * A^(i-j) * B
            let val = (C.clone() * A_pow.clone() * B.clone())[(0, 0)];
            Phi[(i, j)] = val;
        }
    }

    // Luenberger observer
    let L = DMatrix::from_row_slice(2, 1, &[0.00031221, 0.00010908]);
    // Initialize the state estimate
    let mut x_hat = DVector::from_row_slice(&[0.0, 0.0]);

    let mut interval = time::interval(Duration::from_millis(10));
    let mut last_u = 0.0;
    let mut t = 0;
    loop {
        let angle = if (t % 200) < 100 { 0.0 } else { 10.0 };

        // 1. Read the current angle
        let current_angle = servo.batch_read_measurements().await.unwrap().angle as f64;
        let y_k = DVector::from_row_slice(&[current_angle]);

        let start_time = time::Instant::now();

        // 2. Observer update (k+1)
        // x_hat_{k+1} = A x_hat_k + B u_k + L ( y_k - C x_hat_k )
        // We need last control input u_k from the previous iteration; store it somewhere.
        // For the very first iteration, you can assume u_k=0 or something reasonable.
        let x_hat_next = A.clone() * x_hat.clone()
            + B.clone() * last_u
            + L.clone() * (y_k - (C.clone() * x_hat.clone()));
        x_hat = x_hat_next;

        // 3. Use x_hat in the MPC to get the next control input
        let u_next = mpc_first_input(&x_hat, &A, &B, &C, &F, &Phi, &Q_bar, &R_bar, Np, Nc, angle);
        let end_time = time::Instant::now();
        info!("MPC solve time: {:?}", end_time - start_time);

        servo.move_to(u_next as f32).await.unwrap();
        last_u = u_next;

        interval.tick().await;
        t += 1;
    }
}

fn mpc_first_input(
    x0: &DVector<f64>,
    A: &DMatrix<f64>,
    B: &DMatrix<f64>,
    C: &DMatrix<f64>,
    F: &DMatrix<f64>,
    Phi: &DMatrix<f64>,
    Q_bar: &DMatrix<f64>,
    R_bar: &DMatrix<f64>,
    Np: usize,
    Nc: usize,
    r: f64,
) -> f64 {
    let nx = A.nrows();
    let r_bar = DVector::from_element(Np, r);

    // Compute (F*x0 - r_bar)
    let Fx0 = F * x0;
    let Fx0_minus_r = &Fx0 - &r_bar;

    // H = Phi^T * Q_bar * Phi + R_bar
    let H = Phi.transpose() * Q_bar * Phi + R_bar;
    // f = Phi^T * Q_bar * (F*x0 - r_bar)
    let f = Phi.transpose() * Q_bar * Fx0_minus_r;

    // Build the equality constraints E * U = 0 for the control horizon
    // For k = Nc..Np-1: u[k] - u[Nc-1] = 0
    let num_eq = Np - Nc;
    let mut E = DMatrix::zeros(num_eq, Np);
    for (row, k) in (Nc..Np).enumerate() {
        E[(row, k)] = 1.0;
        E[(row, Nc - 1)] = -1.0;
    }
    // We want E * U = 0
    let l_eq = DVector::from_element(num_eq, 0.0);
    let u_eq = DVector::from_element(num_eq, 0.0);

    // OSQP solves:
    //   minimize (1/2) x^T P x + q^T x
    //   subject to l <= A x <= u
    //
    // We'll map:
    //   P = H,  q = f
    //   A = E,  l = 0,  u = 0
    //
    // Also note: OSQP wants (1/2)x^T P x, so we'll pass H directly as P.

    // Convert to OSQP’s data structures
    let P = H; // (Np x Np)
    let q = f; // (Np)
    let A_osqp = E; // (num_eq x Np)
    let l_osqp = l_eq;
    let u_osqp = u_eq;

    info!("P: {:?}", P);
    info!("A_osqp: {:?}", A_osqp);

    let P: [[f64; 10]; 10] = array::from_fn(|i| array::from_fn(|j| P[(i, j)]));
    let P = CscMatrix::from(&P).into_upper_tri();

    let A: [[f64; 10]; 6] = array::from_fn(|i| array::from_fn(|j| A_osqp[(i, j)]));

    // Prepare problem
    let settings = Settings::default().verbose(false);
    let mut problem = Problem::new(
        &P,
        q.as_slice(),
        &A,
        l_osqp.as_slice(),
        u_osqp.as_slice(),
        &settings,
    )
    .expect("Failed to create OSQP problem");

    // Solve
    let result = problem.solve();
    if let Status::Solved(solution) = result {
        // solution.x is the vector U_opt of length Np
        let U_opt = solution.x().to_vec();
        // Return only the first control input
        U_opt[0]
    } else {
        eprintln!("OSQP failed to find a solution.");
        0.0
    }

    // 0.0
}
