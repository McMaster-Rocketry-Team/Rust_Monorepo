use std::{fs, path::PathBuf};

use crate::{
    args::NodeTypeEnum,
    bluetooth::BluetoothConnectionMethod,
    elf_locator::{ElfInfo, find_newest_elf},
    monitor::{MonitorStatus, target_log::TargetLog},
    probe::ProbeConnectionMethod,
    usb::USBConnectionMethod,
};
use anyhow::{Result, anyhow, bail};
use async_trait::async_trait;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use log::{info, warn};
use prompted::input;
use tokio::sync::{broadcast, oneshot, watch};

fn try_read_chip_from_embed_toml() -> Result<String> {
    let path = PathBuf::from("./Embed.toml");
    if !path.exists() {
        bail!("./Embed.toml does not exist")
    }

    let config_str = fs::read_to_string(path)?;
    let config = config_str.parse::<toml::Table>()?;
    let chip = config["default"]["general"]["chip"].as_str();

    chip.map(String::from)
        .ok_or(anyhow!("default.general.chip key not found"))
}

fn try_find_newest_elf() -> Result<ElfInfo> {
    let newest_elf = find_newest_elf(&std::env::current_dir()?)?;
    newest_elf.ok_or(anyhow!("can not find an elf file"))
}

pub async fn get_connection_method(
    chip: Option<String>,
    firmware_elf_path: Option<PathBuf>,
    node_type: Option<NodeTypeEnum>,
    secret_path: Option<PathBuf>,
) -> Result<Box<dyn ConnectionMethod>> {
    // try to auto detect chip
    let chip = if let Some(chip) = chip {
        info!("using chip from args: {}", chip);
        Some(chip)
    } else {
        match try_read_chip_from_embed_toml() {
            Ok(chip) => {
                info!("auto detected chip: {}", chip);
                Some(chip)
            }
            Err(e) => {
                warn!("failed to auto detect chip: {:?}", e);
                None
            }
        }
    };

    // try to auto detect firmware elf path
    let firmware_elf_path = if let Some(firmware_elf_path) = firmware_elf_path {
        info!("using ELF from args: {}", firmware_elf_path.display());
        Some(firmware_elf_path)
    } else {
        match try_find_newest_elf() {
            Ok(elf) => {
                info!(
                    "found ELF: {:<20} built at {}",
                    format!(
                        "{} ({})",
                        elf.path.file_name().unwrap().to_str().unwrap(),
                        elf.profile,
                    ),
                    chrono::DateTime::<chrono::Local>::from(elf.created_time)
                        .format("%Y-%m-%d %H:%M:%S")
                        .to_string()
                );
                Some(elf.path)
            }
            Err(e) => {
                warn!("failed to find elf in current directory: {:?}", e);
                None
            }
        }
    };

    // try to auto detect node type
    let node_type = if let Some(node_type) = node_type {
        info!("using node type from args: {}", node_type);
        node_type
    } else {
        let current_dir = std::env::current_dir()?;
        let parent_dir = current_dir.parent().unwrap();
        let folder_name = parent_dir.file_name().unwrap().to_str().unwrap();

        let node_type = match folder_name {
            "VLF5" => NodeTypeEnum::VoidLake,
            "Titan_AMP" => NodeTypeEnum::AMP,
            "ICARUS" => NodeTypeEnum::ICARUS,
            "OZYS_V3" => NodeTypeEnum::OZYS,
            "Titan_Bulkhead_PCB" => NodeTypeEnum::Bulkhead,
            _ => NodeTypeEnum::Other,
        };
        info!("auto detected node type: {:?}", node_type);
        node_type
    };

    // list all options
    let mut options: Vec<ConnectionOption> = vec![];

    options.append(
        &mut ProbeConnectionMethod::list_options(chip, firmware_elf_path.clone(), node_type)
            .await?,
    );
    options.append(&mut USBConnectionMethod::list_options().await?);
    options.append(
        &mut BluetoothConnectionMethod::list_options(secret_path, firmware_elf_path, node_type)
            .await?,
    );

    if options.len() == 0 {
        bail!("No connection methods found");
    }

    if options.len() == 1 {
        let mut option = options.remove(0);
        info!(
            "using the only avaliable connection method: {}",
            option.name
        );
        let connection_method = option.factory.initialize().await?;
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

    let mut option = options.remove(choice - 1);
    let connection_method = option.factory.initialize().await?;

    Ok(connection_method)
}

pub struct ConnectionOption {
    pub name: String,
    pub factory: Box<dyn ConnectionMethodFactory>,
}

#[async_trait(?Send)]
pub trait ConnectionMethodFactory {
    async fn initialize(&mut self) -> Result<Box<dyn ConnectionMethod>>;
}

#[async_trait(?Send)]
pub trait ConnectionMethod {
    fn name(&self) -> String;

    async fn download(&mut self) -> Result<()>;

    async fn attach(
        &mut self,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()>;

    async fn dispose(&mut self) -> Result<()> {
        Ok(())
    }
}
