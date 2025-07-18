use std::{any::Any, fs, path::PathBuf, process::Stdio};

use crate::{
    args::NodeTypeEnum,
    connection_method::{ConnectionMethod, ConnectionOption},
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
use prompted::input;
use regex::Regex;
use tokio::{
    io::{AsyncBufReadExt as _, BufReader},
    process::Command,
    sync::{broadcast, oneshot, watch},
};

pub struct ProbeConnectionMethod {
    probe_string: String,
}

impl ProbeConnectionMethod {
    fn try_read_chip_from_embed_toml() -> Result<String> {
        // let options = ProbeConnectionMethodOptions {a:1};
        // let options: Box<dyn Any> = Box::new(options);
        // info!("{}", options.as_ref().is::<ProbeConnectionMethodOptions>());
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

    pub async fn list_options(chip: Option<String>) -> Result<Vec<ConnectionOption>> {
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
            info!("no probe connected")
        } else {
            let chip = if let Some(chip) = chip {
                chip
            } else {
                match Self::try_read_chip_from_embed_toml() {
                    Ok(chip) => chip,
                    Err(e) => {
                        info!(
                            "probe options skipped because --chip is not specified and can not read chip from Embed.toml: {:?}",
                            e
                        );
                        return Ok(vec![]);
                    }
                }
            };
        }

        Ok(vec![])
    }

    pub async fn initialize() -> Result<Self> {
        let lister = Lister::new();
        let probes = lister.list_all();
        let probe = if probes.len() == 0 {
            bail!("No probe connected")
        } else if probes.len() == 1 {
            probes[0].clone()
        } else {
            for i in 0..probes.len() {
                let probe = &probes[i];

                println!(
                    "[{}]: {}, SN {}",
                    i + 1,
                    probe.identifier,
                    probe.serial_number.clone().unwrap_or("N/A".into())
                );
            }

            let selection = input!("Select one probe (1-{}): ", probes.len());

            let selection: usize = match selection.trim().parse() {
                Err(_) => bail!("Invalid selection"),
                Ok(num) if num > probes.len() => bail!("Invalid selection"),
                Ok(num) => num,
            };

            probes[selection].clone()
        };

        let probe_string = format!(
            "{:x}:{:x}{}",
            probe.vendor_id,
            probe.product_id,
            probe
                .serial_number
                .map_or(String::new(), |sn| format!(":{}", sn))
        );

        Ok(Self { probe_string })
    }

    pub async fn has_probe_connected() -> bool {
        let lister = Lister::new();
        let probes = lister.list_all();
        !probes.is_empty()
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for ProbeConnectionMethod {
    fn name(&self) -> String {
        self.probe_string.clone()
    }

    async fn download(
        &mut self,
        chip: &String,
        _secret_path: &PathBuf,
        _node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
    ) -> Result<()> {
        let probe_rs_args = [
            "download",
            "--non-interactive",
            "--probe",
            &self.probe_string,
            "--chip",
            chip,
            "--connect-under-reset",
            firmware_elf_path.to_str().unwrap(),
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
        chip: &String,
        _secret_path: &PathBuf,
        node_type: &NodeTypeEnum,
        firmware_elf_path: &PathBuf,
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
            chip,
            "--connect-under-reset",
            "--log-format",
            ">>>>>{s}|||||{F}|||||{l}|||||{L}|||||{m}|||||{t}<<<<<",
            firmware_elf_path.to_str().unwrap(),
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
                        node_type: *node_type,
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
