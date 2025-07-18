use std::{any::Any, path::PathBuf};

use crate::{
    args::NodeTypeEnum,
    bluetooth::BluetoothConnectionMethod,
    monitor::{MonitorStatus, target_log::TargetLog},
    probe::ProbeConnectionMethod,
};
use anyhow::{Result, bail};
use async_trait::async_trait;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use tokio::sync::{broadcast, oneshot, watch};

pub async fn get_connection_method(
    force_bluetooth: bool,
    force_probe: bool,
) -> Result<Box<dyn ConnectionMethod>> {
    let connection_method: Box<dyn ConnectionMethod> = match (force_bluetooth, force_probe) {
        (false, false) => {
            let probe_connected = ProbeConnectionMethod::has_probe_connected().await;

            if probe_connected {
                Box::new(ProbeConnectionMethod::initialize().await?)
            } else {
                Box::new(BluetoothConnectionMethod::initialize().await?)
            }
        }
        (false, true) => Box::new(ProbeConnectionMethod::initialize().await?),
        (true, false) => Box::new(BluetoothConnectionMethod::initialize().await?),
        _ => {
            bail!("--force-bluetooth and --force-probe can not be selected at the same time")
        }
    };

    Ok(connection_method)
}

pub struct ConnectionOption {
    pub name: String,
    pub additional_options: Box<dyn Any>,
}

#[async_trait(?Send)]
pub trait ConnectionMethod {
    fn name(&self) -> String;

    async fn download(
        &mut self,
        chip: &String,
        secret_path: &PathBuf,
        node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
    ) -> Result<()>;

    async fn attach(
        &mut self,
        chip: &String,
        secret_path: &PathBuf,
        node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()>;

    async fn dispose(&mut self) -> Result<()> {
        Ok(())
    }
}
