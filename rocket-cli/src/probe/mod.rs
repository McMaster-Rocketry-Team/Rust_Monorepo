use std::{fs, path::PathBuf, process::Stdio};

use crate::{
    args::NodeTypeEnum,
    connection_method::{ConnectionMethod, ConnectionOption},
    elf_locator::{ElfInfo, find_newest_elf},
    monitor::{
        MonitorStatus,
        target_log::{DefmtLocationInfo, DefmtLogInfo, TargetLog, parse_log_level},
    },
};
use anyhow::{Result, anyhow, bail};
use async_trait::async_trait;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use log::{info, warn};
use probe_rs::probe::list::Lister;
use regex::Regex;
use tokio::{
    io::{AsyncBufReadExt as _, BufReader},
    process::Command,
    sync::{broadcast, oneshot, watch},
};

pub struct ProbeConnectionMethod {
    chip: String,
    firmware_elf_path: PathBuf,
    node_type: NodeTypeEnum,
    probe_string: String,
}

impl ProbeConnectionMethod {
    fn try_read_chip_from_embed_toml() -> Result<String> {
        let path = PathBuf::from("./Embed.toml");
        if !path.exists() {
            bail!("./Embed.toml does not exist")
        }

        let config_str = fs::read_to_string(path)?;
        let config = config_str.parse::<toml::Table>()?;
        info!("{:?}", config);
        let chip = config["default"]["general"]["chip"].as_str();

        chip.map(String::from)
            .ok_or(anyhow!("default.general.chip key not found"))
    }

    fn try_find_newest_elf() -> Result<ElfInfo> {
        let newest_elf = find_newest_elf(&".")?;
        newest_elf.ok_or(anyhow!("can not find an elf file"))
    }

    pub async fn list_options(
        chip: Option<String>,
        firmware_elf_path: Option<std::path::PathBuf>,
    ) -> Result<Vec<ConnectionOption>> {
        let output = std::process::Command::new("probe-rs")
            .arg("--version")
            .output();

        if output.is_err() {
            warn!(
                "probe-rs not found. Please install it by running 'cargo install probe-rs-tools --locked'"
            );
            return Ok(vec![]);
        }

        let lister = Lister::new();
        let probes = lister.list_all();
        if probes.len() == 0 {
            info!("no probe connected");
            return Ok(vec![]);
        }

        // find chip part number
        let chip = if let Some(chip) = chip {
            info!("using chip from args: {}", chip);
            chip
        } else {
            match Self::try_read_chip_from_embed_toml() {
                Ok(chip) => {
                    info!("auto detected chip: {}", chip);
                    chip
                }
                Err(e) => {
                    info!(
                        "probe options skipped because --chip is not specified and can not read chip from Embed.toml: {:?}",
                        e
                    );
                    return Ok(vec![]);
                }
            }
        };

        // find elf file
        let firmware_elf_path = if let Some(firmware_elf_path) = firmware_elf_path {
            info!("using ELF from args: {}", firmware_elf_path.display());
            firmware_elf_path
        } else {
            match Self::try_find_newest_elf() {
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
                    elf.path
                }
                Err(e) => {
                    info!(
                        "probe options skipped because --elf is not specified and can not find elf in current project: {:?}",
                        e
                    );
                    return Ok(vec![]);
                }
            }
        };

        // find node type
        let current_dir = std::env::current_dir()?;
        let folder_name = current_dir.file_name().unwrap().to_str().unwrap();

        // Try to infer node type from folder name
        let node_type = match folder_name {
            "VLF5" => NodeTypeEnum::VoidLake,
            "Titan_AMP" => NodeTypeEnum::AMP,
            "ICARUS" => NodeTypeEnum::ICARUS,
            "OZYS_V3" => NodeTypeEnum::OZYS,
            "Titan_Bulkhead_PCB" => NodeTypeEnum::Bulkhead,
            _ => NodeTypeEnum::Other,
        };
        info!("auto detected node type: {:?}", node_type);

        // list probes
        let mut options = vec![];

        for i in 0..probes.len() {
            let probe = &probes[i];

            let chip = chip.clone();
            let firmware_elf_path = firmware_elf_path.clone();
            let node_type = node_type.clone();

            let probe_string = format!(
                "{:x}:{:x}{}",
                probe.vendor_id,
                probe.product_id,
                probe
                    .serial_number
                    .clone()
                    .map_or(String::new(), |sn| format!(":{}", sn))
            );

            options.push(ConnectionOption {
                name: format!(
                    "Probe {}, SN {}",
                    probe.identifier,
                    probe.serial_number.clone().unwrap_or("N/A".into())
                ),
                initializer: Box::new(move || {
                    let probe_connection = Self {
                        chip,
                        firmware_elf_path,
                        node_type,
                        probe_string,
                    };
                    Ok(Box::new(probe_connection))
                }),
            });
        }

        Ok(options)
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for ProbeConnectionMethod {
    fn name(&self) -> String {
        self.probe_string.clone()
    }

    async fn download(
        &mut self,
        _chip: &String,
        _secret_path: &PathBuf,
        _node_type: &NodeTypeEnum,
        _firmware_elf_path: &PathBuf,
    ) -> Result<()> {
        let probe_rs_args = [
            "download",
            "--non-interactive",
            "--probe",
            &self.probe_string,
            "--chip",
            &self.chip,
            "--connect-under-reset",
            &self.firmware_elf_path.to_str().unwrap(),
        ];
        let output = std::process::Command::new("probe-rs")
            .args(&probe_rs_args)
            .status()?;

        if !output.success() {
            bail!("probe-rs command failed");
        }

        Ok(())
    }

    async fn attach(
        &mut self,
        _chip: &String,
        _secret_path: &PathBuf,
        _node_type: &NodeTypeEnum,
        _firmware_elf_path: &PathBuf,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        _messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        let probe_rs_args = [
            "attach",
            "--non-interactive",
            "--probe",
            &self.probe_string,
            "--chip",
            &self.chip,
            "--connect-under-reset",
            "--log-format",
            ">>>>>{s}|||||{F}|||||{l}|||||{L}|||||{m}|||||{t}<<<<<",
            &self.firmware_elf_path.to_str().unwrap(),
        ];

        let mut child = Command::new("probe-rs")
            .args(&probe_rs_args)
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()?;

        let stdout = child.stdout.take().unwrap();
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();
        let re = Regex::new(r">>>>>(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)<<<<<").unwrap();
        status_tx.send(MonitorStatus::Normal).ok();

        let read_logs_future = async move {
            while let Some(line) = lines.next_line().await.unwrap() {
                if let Some(cap) = re.captures(&line) {
                    let log = TargetLog {
                        node_type: self.node_type,
                        node_id: None,
                        log_content: cap.get(1).unwrap().as_str().to_string(),
                        defmt: Some(DefmtLogInfo {
                            location: Some(DefmtLocationInfo {
                                file_path: cap.get(2).unwrap().as_str().to_string(),
                                line_number: cap.get(3).unwrap().as_str().to_string(),
                                module_path: cap.get(5).unwrap().as_str().to_string(),
                            }),
                            log_level: parse_log_level(cap.get(4).unwrap().as_str()),
                            timestamp: cap.get(6).unwrap().as_str().parse::<f64>().ok(),
                        }),
                    };
                    logs_tx.send(log).ok();
                }
            }
        };

        tokio::select! {
            _ = read_logs_future => {},
            _ = stop_rx => {},
        }

        child.kill().await?;

        Ok(())
    }
}
