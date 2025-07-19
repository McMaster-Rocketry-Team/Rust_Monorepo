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
use prompted::input;
use tokio::sync::{broadcast, oneshot, watch};

pub async fn get_connection_method(
    chip: Option<String>,
    firmware_elf_path: Option<PathBuf>,
) -> Result<Box<dyn ConnectionMethod>> {
    let mut options: Vec<ConnectionOption> = vec![];

    options.append(&mut ProbeConnectionMethod::list_options(chip, firmware_elf_path).await?);


    if options.len() == 0 {
        bail!("No connection methods found");
    }

    if options.len() == 1 {
        let option = options.remove(0);
        let connection_method = (option.initializer)()?;
        return Ok(connection_method);
    }

    for i in 0..options.len() {
        let option = &options[i];
        println!("[{}]: {}", i + 1, option.name);
    }

    let choice = input!("Select one connection method: (1-{})", options.len());
    let choice = choice.parse::<usize>()?;
    if choice < 1 || choice > options.len() {
        bail!("Invalid choice");
    }

    let option = options.remove(choice - 1);
    let connection_method = (option.initializer)()?;

    Ok(connection_method)
}

pub struct ConnectionOption {
    pub name: String,
    pub initializer: Box<dyn FnOnce() -> Result<Box<dyn ConnectionMethod>>>,
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
