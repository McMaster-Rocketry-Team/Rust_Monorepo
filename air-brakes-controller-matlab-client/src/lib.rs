use std::array;

use air_brakes_controller_matlab_service::{AirBrakesControllerServiceClient, SERVER_ADDR};
use rustmex::prelude::*;
use rustmex::{
    MatlabClass,
    convert::ToMatlab,
    numeric::{Numeric, NumericArray},
};
use tarpc::tokio_serde::formats::Json;
use tarpc::{client, context};

async fn update(reset: bool, acc: [f32; 3], gyro: [f32; 3], servo_angle: f32) -> f32 {
    let tcp = tarpc::serde_transport::tcp::connect(&SERVER_ADDR, Json::default);
    let transport = tcp.await.unwrap();

    let client =
        AirBrakesControllerServiceClient::new(client::Config::default(), transport).spawn();

    client
        .update(context::current(), reset, acc, gyro, servo_angle)
        .await
        .unwrap()
}

#[rustmex::entrypoint]
fn air_brakes_controller(lhs: Lhs, rhs: Rhs) -> rustmex::Result<()> {
    let reset = Numeric::<f64, _>::from_mx_array(rhs[0])?.data();
    let reset = reset[0] != 0.0;

    let acc = Numeric::<f64, _>::from_mx_array(rhs[1])?.data();
    let acc: [f32; 3] = array::from_fn(|i| acc[i] as f32);

    let gyro = Numeric::<f64, _>::from_mx_array(rhs[2])?.data();
    let gyro: [f32; 3] = array::from_fn(|i| gyro[i] as f32);

    let servo_angle = Numeric::<f64, _>::from_mx_array(rhs[3])?.data();
    let servo_angle = servo_angle[0] as f32;

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    let sum = rt.block_on(update(reset, acc, gyro, servo_angle)) as f64;

    lhs[0].replace(sum.to_matlab());

    Ok(())
}
