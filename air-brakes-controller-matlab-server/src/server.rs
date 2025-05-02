use std::sync::{Arc, Mutex};

use air_brakes_controller_matlab_service::{AirBrakesControllerService, SERVER_ADDR};
use futures::prelude::*;
use log::{LevelFilter, info};
use tarpc::{
    context,
    server::{self, Channel},
    tokio_serde::formats::Json,
};

#[derive(Debug)]
struct AirBrakesControllerServerState {
    state: f32,
}

impl AirBrakesControllerServerState {
    fn new() -> Self {
        AirBrakesControllerServerState { state: 0.0 }
    }
}

#[derive(Clone)]
struct AirBrakesControllerServer(Arc<Mutex<AirBrakesControllerServerState>>);

impl AirBrakesControllerService for AirBrakesControllerServer {
    async fn update(
        self,
        _: context::Context,
        reset: bool,
        acc: [f32; 3],
        gyro: [f32; 3],
        servo_angle: f32,
    ) -> f32 {
        let mut state = self.0.lock().unwrap();
        info!(
            "Received update request with acc: {:?}, gyro: {:?}, servo_angle: {}, state: {:?}",
            acc, gyro, servo_angle, *state
        );

        if reset {
            *state = AirBrakesControllerServerState::new();
        }

        state.state += 1.0;
        // air_brakes_controller_core::add(a, b)
        1.23
    }
}

// code from https://github.com/google/tarpc/blob/master/example-service/src/server.rs
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .filter_level(LevelFilter::Warn)
        .filter(
            Some("air_brakes_controller_matlab_server"),
            LevelFilter::Trace,
        )
        .is_test(false)
        .try_init()
        .unwrap();

    let listener = tarpc::serde_transport::tcp::listen(&SERVER_ADDR, Json::default).await?;
    info!("Listening on port {}", listener.local_addr().port());

    let state = Arc::new(Mutex::new(AirBrakesControllerServerState::new()));

    listener
        .filter_map(async |r| r.ok())
        .map(server::BaseChannel::with_defaults)
        .map(move |channel| {
            let state = Arc::clone(&state);
            let server = AirBrakesControllerServer(state);
            channel.execute(server.serve()).for_each(spawn)
        })
        .buffer_unordered(1)
        .for_each(|_| async {})
        .await;

    Ok(())
}

async fn spawn(fut: impl Future<Output = ()> + Send + 'static) {
    tokio::spawn(fut);
}
