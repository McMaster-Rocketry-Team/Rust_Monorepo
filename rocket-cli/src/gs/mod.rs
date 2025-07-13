use std::io::Write;

use anyhow::Result;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use firmware_common_new::{
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPGroundStation,
        lora_config::LoraConfig,
        packets::fire_pyro::{FirePyroPacket, PyroSelect},
        radio::Radio,
    },
};
use futures::FutureExt;
use log::{error, info, warn};
use lora_phy::mod_params::{PacketStatus, RadioError};
use rustyline_async::{Readline, ReadlineEvent};
use tokio_serial::SerialPortBuilderExt as _;

use crate::gs::serial_wrapper::{Delay, SerialWrapper};

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
            frequency: 916_000_000,
            sf: 11,
            bw: 250_000,
            cr: 8,
            power: 0,
        })
        .await
        .unwrap();

    let mut rpc_radio = RpcRadio::new(client);

    let vlp_gcm_client = VLPGroundStation::<NoopRawMutex>::new();

    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &VLP_KEY);

    let daemon_fut = daemon.run();

    let (mut rl, mut writer) =
        Readline::new("Select pyro to fire (main, drogue): ".to_owned()).unwrap();

    let print_telemetry_fut = async {
        loop {
            let (packet, status) = vlp_gcm_client.receive().await;
            writer
                .write_all(
                    format!("{:?} rssi={} snr={}\n", packet, status.rssi, status.snr).as_bytes(),
                )
                .unwrap();
        }
    };

    let fire_pyro_fut = async {
        loop {
            match rl.readline().fuse().await {
                Ok(ReadlineEvent::Line(line)) => match line.trim() {
                    "main" => {
                        info!("sending.....");
                        let result = vlp_gcm_client
                            .send(
                                FirePyroPacket {
                                    pyro: PyroSelect::Pyro1,
                                }
                                .into(),
                            )
                            .await;
                        info!("{:?}", result.err());
                    }
                    "drogue" => {
                        info!("sending.....");
                        let result = vlp_gcm_client
                            .send(
                                FirePyroPacket {
                                    pyro: PyroSelect::Pyro2,
                                }
                                .into(),
                            )
                            .await;
                        info!("{:?}", result.err());
                    }
                    _ => {
                        warn!("invalid selection")
                    }
                },
                e => {
                    warn!("{:?}", e);
                }
            }
        }
    };

    tokio::join!(daemon_fut, print_telemetry_fut, fire_pyro_fut,);

    Ok(())
}

struct RpcRadio<'a> {
    client: LoraRpcClient<'a, SerialWrapper, Delay>,
    buffer: [u8; 256],
}

impl<'a> RpcRadio<'a> {
    fn new(client: LoraRpcClient<'a, SerialWrapper, Delay>) -> Self {
        Self {
            client,
            buffer: [0u8; 256],
        }
    }
}

impl<'a> Radio for RpcRadio<'a> {
    async fn tx(&mut self, buffer: &[u8]) -> std::result::Result<(), RadioError> {
        self.buffer[..buffer.len()].copy_from_slice(buffer);
        match self.client.tx(buffer.len() as u32, self.buffer).await {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("{:?}", e);
                Err(RadioError::TransmitTimeout)
            }
        }
    }

    async fn rx(
        &mut self,
        buffer: &mut [u8],
        timeout_ms: Option<u16>,
    ) -> std::result::Result<(usize, PacketStatus), RadioError> {
        if let Some(timeout_ms) = timeout_ms {
            match self.client.rx(timeout_ms as u32).await {
                Ok(response) => {
                    buffer[..(response.len as usize)]
                        .copy_from_slice(&response.data[..(response.len as usize)]);

                    Ok((
                        response.len as usize,
                        PacketStatus {
                            rssi: response.status.rssi,
                            snr: response.status.snr,
                        },
                    ))
                }
                Err(e) => {
                    error!("{:?}", e);
                    Err(RadioError::ReceiveTimeout)
                }
            }
        } else {
            loop {
                match self.client.rx(4000).await {
                    Ok(response) => {
                        buffer[..(response.len as usize)]
                            .copy_from_slice(&response.data[..(response.len as usize)]);

                        return Ok((
                            response.len as usize,
                            PacketStatus {
                                rssi: response.status.rssi,
                                snr: response.status.snr,
                            },
                        ));
                    }
                    Err(e) => {
                        error!("{:?}", e);
                    }
                }
            }
        }
    }
}
