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
use log::{Level, info, warn};
use probe_rs::probe::list::Lister;
use regex::Regex;
use tokio::{
    io::{AsyncBufReadExt as _, AsyncReadExt, BufReader},
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

    async fn download(&mut self) -> Result<()> {
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
            .stderr(Stdio::piped())
            .spawn()?;

        let stdout = child.stdout.take().unwrap();
        let stderr = child.stderr.take().unwrap();
        let mut reader = BufReader::new(stdout);
        let mut stdout_buffer = String::new();
        let stderr_reader = BufReader::new(stderr);
        let mut stderr_lines = stderr_reader.lines();
        let mut stderr = String::new();
        let re = Regex::new(r">>>>>((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))<<<<<").unwrap();
        status_tx.send(MonitorStatus::Normal).ok();

        let node_type = self.node_type;
        let logs_tx2 = logs_tx.clone();
        let stdout_fut = async {
            let read_logs_future = async move {
                while let Ok(c) = reader.read_u8().await {
                    stdout_buffer.push(c as char);
                    if stdout_buffer.ends_with("\n") && !stdout_buffer.starts_with(">>>>>"){
                        let log = TargetLog {
                            node_type: node_type,
                            node_id: None,
                            log_content: stdout_buffer[0..(stdout_buffer.len()-1)].into(),
                            defmt: Some(DefmtLogInfo {
                                location: None,
                                log_level: Level::Warn,
                                timestamp: None,
                            }),
                        };
                        logs_tx2.send(log).ok();
                        stdout_buffer.clear();
                    }
                    if stdout_buffer.ends_with(">>>>>") {
                        stdout_buffer = ">>>>>".into();
                    }
                    if stdout_buffer.ends_with("<<<<<") {
                        if let Some(cap) = re.captures(&stdout_buffer) {
                            let log = TargetLog {
                                node_type: node_type,
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
                            logs_tx2.send(log).ok();
                        }
                        stdout_buffer.clear();
                    }
                }
            };

            tokio::select! {
                _ = read_logs_future => {
                    info!("read_logs_future stopped");
                },
                _ = stop_rx => {
                    info!("stop_rx stopped");
                },
            }

            child.kill().await.ok();
        };

        let stderr_fut = async {
            while let Some(line) = stderr_lines.next_line().await.unwrap() {
                warn!("stderr: {}", line);
                stderr.push_str(&line);
                stderr.push('\n');
            }
            info!("stderr_fut stopped");
        };

        // let exit_status_fut = async {
        //     loop {
        //         sleep(Duration::from_millis(500)).await;
        //         let result = child.try_wait();
        //         info!("try wait: {:?}", result);
        //         if let Ok(Some(status)) = result {
        //             let log = TargetLog {
        //                 node_type: self.node_type,
        //                 node_id: None,
        //                 log_content: format!("probe-rs existed with status {}", status),
        //                 defmt: Some(DefmtLogInfo {
        //                     location: None,
        //                     log_level: Level::Warn,
        //                     timestamp: None,
        //                 }),
        //             };
        //             logs_tx.send(log).ok();
        //             break;
        //         };
        //     }
        // };

        tokio::join!(stdout_fut, stderr_fut);

        if !stderr.is_empty() {
            warn!("stderr output from probe-run:\n{}", stderr);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_defmt_log_regex() {
        let re = Regex::new(r">>>>>((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))\|\|\|\|\|((?s:.*?))<<<<<").unwrap();

        let test_log =
            ">>>>>test\n\rmessage|||||src/main.rs|||||123|||||INFO|||||my_module|||||1.234<<<<<";
        let captures = re.captures(test_log).unwrap();

        assert_eq!(captures.get(1).unwrap().as_str(), "test\n\rmessage");
        assert_eq!(captures.get(2).unwrap().as_str(), "src/main.rs");
        assert_eq!(captures.get(3).unwrap().as_str(), "123");
        assert_eq!(captures.get(4).unwrap().as_str(), "INFO");
        assert_eq!(captures.get(5).unwrap().as_str(), "my_module");
        assert_eq!(captures.get(6).unwrap().as_str(), "1.234");

        // Test that regex doesn't match invalid format
        let invalid_log = ">>>>missing separators and closing<<<<<";
        assert!(re.captures(invalid_log).is_none());
    }
}
