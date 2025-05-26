use std::{hint::black_box, path::PathBuf, time::Duration};

use crate::args::NodeTypeEnum;
use crate::monitor::MonitorStatus;
use crate::monitor::target_log::TargetLog;
use crate::{connection_method::ConnectionMethod, elf_locator::locate_elf_files};
use anyhow::{Result, anyhow, bail};
use async_trait::async_trait;
use ble_download::ble_download;
use btleplug::api::{Central as _, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::Manager;
use btleplug::platform::Peripheral;
use demultiplex_log::LogDemultiplexer;
use extract_bin::extract_bin_and_sign;
use firmware_common_new::can_bus::telemetry::message_aggregator::{
    DecodedMessage, decode_aggregated_can_bus_messages,
};
use log::{debug, info, warn};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
};

mod ble_download;
pub mod demultiplex_log;
mod extract_bin;

pub struct BluetoothConnectionMethod {
    peripheral: Peripheral,
}

impl BluetoothConnectionMethod {
    pub async fn initialize() -> Result<Self> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let adapter = if adapters.len() == 0 {
            bail!("No bluetooth adapter found")
        } else if adapters.len() == 1 {
            info!("Found 1 bluetooth adapter");
            adapters[0].clone()
        } else {
            info!(
                "Found {} bluetooth adapters, using the first one",
                adapters.len()
            );
            adapters[0].clone()
        };

        adapter.start_scan(ScanFilter::default()).await?;
        info!("Searching for RocketOTA.....");

        let mut count = 0;
        loop {
            let peripherals = adapter.peripherals().await?;
            for peripheral in peripherals {
                let properties = peripheral.properties().await;
                // info!("{:?} {:?}", peripheral, properties);
                if let Ok(Some(properties)) = properties {
                    if properties.local_name == Some("RocketOTA".into()) {
                        peripheral.connect().await?;
                        peripheral.discover_services().await?;
                        return Ok(Self { peripheral });
                    }
                }
            }

            count += 1;
            if count > 30 {
                bail!("ESP not found");
            }
            sleep(Duration::from_secs(1)).await;
        }
    }

    pub fn process_chunk(
        chunk: &[u8],
        log_demultiplexer: &mut LogDemultiplexer,
        logs_tx: &broadcast::Sender<TargetLog>,
        messages_tx: &broadcast::Sender<DecodedMessage>,
    ) -> Result<bool> {
        if chunk.len() == 0 {
            bail!("Chunk too short");
        }

        let chunk_type = chunk[0] >> 6;
        let is_overrun = match chunk_type {
            0b00 => {
                debug!("received log multiplexer chunk");
                log_demultiplexer.process_chunk(chunk, logs_tx)?
            }
            0b01 => {
                debug!("received aggregated message chunk");
                decode_aggregated_can_bus_messages(chunk, |message| {
                    messages_tx.send(message).ok();
                })
                .map_err(|e| anyhow!("{:?}", e))?
            }
            typ => bail!("Invalid chunk type: {}", typ),
        };

        Ok(is_overrun)
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for BluetoothConnectionMethod {
    fn name(&self) -> String {
        String::from("RocketOTA")
    }

    async fn download(
        &mut self,
        _chip: &String,
        secret_path: &PathBuf,
        node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
    ) -> Result<()> {
        let firmware_bytes = extract_bin_and_sign(secret_path, firmware_elf_path).await?;
        ble_download(&firmware_bytes, *node_type, &self.peripheral).await?;
        Ok(())
    }

    async fn attach(
        &mut self,
        _chip: &String,
        _secret_path: &PathBuf,
        _node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        let elf_info_map = locate_elf_files(Some(firmware_elf_path))
            .map_err(|e| warn!("{:?}", e))
            .unwrap_or_default();
        let mut log_demultiplexer = LogDemultiplexer::new(elf_info_map);

        // sleep for 1 sec so we have time to see the logs before monitor takes over
        sleep(Duration::from_secs(1)).await;

        let receive_future = async {
            loop {
                let chunk: &[u8] = black_box(&[]); // TODO

                let status = match Self::process_chunk(
                    chunk,
                    &mut log_demultiplexer,
                    &logs_tx,
                    &messages_tx,
                ) {
                    Ok(false) => MonitorStatus::Normal,
                    Ok(true) => MonitorStatus::Overrun,
                    Err(_) => MonitorStatus::ChunkError,
                };
                status_tx.send(status).ok();
            }
        };

        tokio::select! {
            _ = receive_future => {}
            _ = stop_rx => {}
        }

        todo!()
    }

    async fn dispose(&mut self) -> Result<()> {
        if self.peripheral.is_connected().await? {
            self.peripheral.disconnect().await?;
        }
        Ok(())
    }
}
