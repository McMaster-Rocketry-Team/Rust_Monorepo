use std::{path::PathBuf, time::Duration};

use crate::args::NodeTypeEnum;
use crate::bluetooth::extract_bin::{extract_bin_from_elf, sign_firmware_binary};
use crate::bluetooth::payload_activation_pcb::PayloadActivationPCB;
use crate::connection_method::{ConnectionMethodFactory, ConnectionOption};
use crate::monitor::MonitorStatus;
use crate::monitor::target_log::TargetLog;
use crate::{connection_method::ConnectionMethod, elf_locator::locate_elf_files};
use anyhow::{Result, anyhow, bail};
use async_trait::async_trait;
use btleplug::api::{Central as _, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::{Adapter, Manager};
use demultiplex_log::LogDemultiplexer;
use firmware_common_new::can_bus::telemetry::log_multiplexer::decode_multiplexed_log_chunk;
use firmware_common_new::can_bus::telemetry::message_aggregator::{
    DecodedMessage, decode_aggregated_can_bus_messages,
};
use log::{info, warn};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
};

pub mod demultiplex_log;
mod extract_bin;
mod payload_activation_pcb;

struct BluetoothConnectionMethodFactory {
    adapter: Adapter,
    bluetooth_name: String,
    secret_path: Option<PathBuf>,
    firmware_path: Option<BluetoothFirmwareType>,
    node_type: NodeTypeEnum,
}

#[async_trait(?Send)]
impl ConnectionMethodFactory for BluetoothConnectionMethodFactory {
    async fn initialize(&mut self) -> Result<Box<dyn ConnectionMethod>> {
        let elf_info_map = locate_elf_files(match self.firmware_path.clone() {
            Some(BluetoothFirmwareType::Elf(path)) => Some(path),
            _ => None,
        })
        .map_err(|e| warn!("{:?}", e))
        .unwrap_or_default();
        let log_demultiplexer = LogDemultiplexer::new(elf_info_map);

        self.adapter.start_scan(ScanFilter::default()).await?;
        info!("Searching for {}.....", self.bluetooth_name);

        let mut count = 0;
        loop {
            let peripherals = self.adapter.peripherals().await?;
            for peripheral in peripherals {
                let properties = peripheral.properties().await;
                // info!("{:?} {:?}", peripheral, properties);
                if let Ok(Some(properties)) = properties {
                    if properties.local_name == Some(self.bluetooth_name.clone()) {
                        peripheral.connect().await?;

                        return Ok(Box::new(BluetoothConnectionMethod {
                            pab: PayloadActivationPCB::new(peripheral).await?,
                            secret_path: self.secret_path.clone(),
                            firmware_path: self.firmware_path.clone(),
                            node_type: self.node_type,
                            log_demultiplexer,
                        }));
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
}

pub struct BluetoothConnectionMethod {
    pab: PayloadActivationPCB,
    secret_path: Option<PathBuf>,
    firmware_path: Option<BluetoothFirmwareType>,
    node_type: NodeTypeEnum,
    log_demultiplexer: LogDemultiplexer,
}

#[derive(Debug, Clone)]
pub enum BluetoothFirmwareType {
    Bin(PathBuf),
    Elf(PathBuf),
}

impl BluetoothConnectionMethod {
    pub async fn list_options(
        secret_path: Option<PathBuf>,
        firmware_path: Option<BluetoothFirmwareType>,
        node_type: NodeTypeEnum,
    ) -> Result<Vec<ConnectionOption>> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let bluetooth_name = match node_type {
            NodeTypeEnum::RocketWifi => "RocketWifi",
            NodeTypeEnum::EPS1 => "RocketWifi",
            NodeTypeEnum::EPS2 => "RocketWifi",
            _ => "RocketOTA",
        };

        let mut options = vec![];

        for adapter in adapters {
            let name = adapter.adapter_info().await?;
            options.push(ConnectionOption {
                name: format!("{}, Bluetooth {}", bluetooth_name, name),
                factory: Box::new(BluetoothConnectionMethodFactory {
                    adapter,
                    bluetooth_name: bluetooth_name.into(),
                    secret_path: secret_path.clone(),
                    firmware_path: firmware_path.clone(),
                    node_type: node_type.clone(),
                }),
                attach_only: secret_path.is_none() || firmware_path.is_none(),
            });
        }

        Ok(options)
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
            0b00 => decode_multiplexed_log_chunk(chunk, |frame| {
                log_demultiplexer.process_frame(frame, logs_tx);
            })
            .map_err(|e: firmware_common_new::can_bus::telemetry::log_multiplexer::DecodeMultiplexedLogError| anyhow!("{:?}", e))?,
            0b01 => decode_aggregated_can_bus_messages(chunk, |message| {
                messages_tx.send(message).ok();
            })
            .map_err(|e| anyhow!("{:?}", e))?,
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

    async fn download(&mut self) -> Result<()> {
        if let Some(secret_path) = self.secret_path.clone()
            && let Some(firmware_path) = self.firmware_path.clone()
        {
            let mut firmware_bin = match firmware_path {
                BluetoothFirmwareType::Bin(path_buf) => std::fs::read(&path_buf)?,
                BluetoothFirmwareType::Elf(path_buf) => extract_bin_from_elf(&path_buf).await?,
            };

            sign_firmware_binary(&mut firmware_bin, &secret_path).await?;

            self.pab.ota(&firmware_bin, self.node_type).await?;
        } else {
            warn!("Bluetooth connection method is not configured for download, skipping");
            sleep(Duration::from_secs(1)).await;
        }
        Ok(())
    }

    async fn attach(
        &mut self,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        // clear outdated data from log_rx
        while let Ok(_) = self.pab.log_rx.try_recv() {}
        info!("waiting for logs from bluetooth.....");

        let receive_future = async {
            while let Some(chunk) = self.pab.log_rx.recv().await {
                let status = match Self::process_chunk(
                    &chunk,
                    &mut self.log_demultiplexer,
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

        Ok(())
    }
}
