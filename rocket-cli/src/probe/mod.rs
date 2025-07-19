use std::{path::PathBuf, process::Stdio};

use crate::{
    args::NodeTypeEnum,
    connection_method::{ConnectionMethod, ConnectionMethodFactory, ConnectionOption},
    monitor::{
        MonitorStatus,
        target_log::{DefmtLocationInfo, DefmtLogInfo, TargetLog, parse_log_level},
    },
};
use anyhow::{Result, bail};
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

struct ProbeConnectionMethodFactory {
    chip: String,
    firmware_elf_path: PathBuf,
    node_type: NodeTypeEnum,
    probe_string: String,
}

#[async_trait(?Send)]
impl ConnectionMethodFactory for ProbeConnectionMethodFactory {
    async fn initialize(&mut self) -> Result<Box<dyn ConnectionMethod>> {
        Ok(Box::new(ProbeConnectionMethod {
            chip: self.chip.clone(),
            firmware_elf_path: self.firmware_elf_path.clone(),
            node_type: self.node_type.clone(),
            probe_string: self.probe_string.clone(),
        }))
    }
}

pub struct ProbeConnectionMethod {
    chip: String,
    firmware_elf_path: PathBuf,
    node_type: NodeTypeEnum,
    probe_string: String,
}

impl ProbeConnectionMethod {
    pub async fn list_options(
        chip: Option<String>,
        firmware_elf_path: Option<std::path::PathBuf>,
        node_type: NodeTypeEnum,
    ) -> Result<Vec<ConnectionOption>> {
        let output = std::process::Command::new("probe-rs")
            .arg("--version")
            .output();

        if output.is_err() {
            warn!(
                "probe-rs not found. Please install it by running 'cargo install probe-rs-tools --locked --force'"
            );
            return Ok(vec![]);
        }

        let lister = Lister::new();
        let probes = lister.list_all();
        if probes.len() == 0 {
            info!("no probe connected");
            return Ok(vec![]);
        }

        let chip = if let Some(chip) = chip {
            chip
        } else {
            info!("probe options skipped because chip is unknown");
            return Ok(vec![]);
        };

        let firmware_elf_path = if let Some(firmware_elf_path) = firmware_elf_path {
            firmware_elf_path
        } else {
            info!("probe options skipped because ELF is unknown");
            return Ok(vec![]);
        };

        // list probes
        let mut options = vec![];

        for i in 0..probes.len() {
            let probe = &probes[i];

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
                factory: Box::new(ProbeConnectionMethodFactory {
                    chip: chip.clone(),
                    firmware_elf_path: firmware_elf_path.clone(),
                    node_type: node_type.clone(),
                    probe_string: probe_string.clone(),
                }),
                attach_only: false,
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
