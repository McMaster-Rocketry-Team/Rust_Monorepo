mod rpc_radio;

use anyhow::Result;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use firmware_common_new::{
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPGroundStation,
        lora_config::LoraConfig,
        packets::fire_pyro::{FirePyroPacket, PyroSelect},
    },
};
use log::{error, info, warn};
use tokio_serial::SerialPortBuilderExt as _;

use crate::gs::{
    rpc_radio::RpcRadio,
    serial_wrapper::{Delay, SerialWrapper},
};

mod serial_wrapper;

const VLP_KEY: [u8; 32] = [42u8; 32];

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
            frequency: 915_100_000,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        })
        .await
        .unwrap();

    let mut rpc_radio = RpcRadio::new(client);

    let vlp_gcm_client = VLPGroundStation::<NoopRawMutex>::new();

    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &VLP_KEY);

    let daemon_fut = daemon.run();

    // let (mut rl, mut writer) =
    //     Readline::new("Select pyro to fire (main, drogue): ".to_owned()).unwrap();

    let print_telemetry_fut = async {
        loop {
            let (packet, status) = vlp_gcm_client.receive().await;
            info!("{:?} rssi={} snr={}\n", packet, status.rssi, status.snr);
        }
    };

    let test_fut = async {
        vlp_gcm_client
            .send(
                FirePyroPacket {
                    pyro: PyroSelect::Pyro1,
                }
                .into(),
            )
            .await
            .unwrap();
    };

    tokio::join!(daemon_fut, print_telemetry_fut, test_fut);

    Ok(())
}
