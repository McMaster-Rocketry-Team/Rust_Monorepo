use anyhow::Result;
use firmware_common_new::{rpc::lora_rpc::LoraRpcClient, vlp::lora_config::LoraConfig};
use log::info;
use tokio_serial::SerialPortBuilderExt as _;

use crate::gs::serial_wrapper::{Delay, SerialWrapper};

mod serial_wrapper;

pub async fn ground_station_tui(serial_port: &str) -> Result<()> {
    let serial = tokio_serial::new(serial_port, 115200)
        .open_native_async()
        .expect("open serial port");
    let mut serial = SerialWrapper(serial);

    let mut client = LoraRpcClient::new(&mut serial, Delay);

    client.reset().await.unwrap();

    info!("resetted");

    client
        .configure(LoraConfig {
            frequency: 916_000_000,
            sf: 11,
            bw: 250_000,
            cr: 8,
            power: 0,
        })
        .await
        .unwrap();

    Ok(())
}
