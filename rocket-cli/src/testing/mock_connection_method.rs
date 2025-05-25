use std::{path::PathBuf, time::Duration};

use crate::{
    args::NodeTypeEnum,
    connect_method::ConnectionMethod,
    monitor::{
        MonitorStatus,
        target_log::{DefmtLocationInfo, DefmtLogInfo, TargetLog},
    },
};
use anyhow::Result;
use async_trait::async_trait;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use log::{Level, info};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
};

pub struct MockConnectionMethod;

#[async_trait(?Send)]
impl ConnectionMethod for MockConnectionMethod {
    fn name(&self) -> String {
        String::from("Mock Connection")
    }

    async fn download(
        &mut self,
        _chip: &String,
        _secret_path: &PathBuf,
        _node_type: &NodeTypeEnum,
        _firmware_elf_path: &PathBuf,
    ) -> Result<()> {
        info!("Downloading.....");
        sleep(Duration::from_secs(1)).await;
        info!("Download done");
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
        mut stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        info!("Attaching.....");
        sleep(Duration::from_millis(500)).await;
        status_tx.send(MonitorStatus::Normal)?;

        loop {
            logs_tx
                .send(TargetLog {
                    node_type: NodeTypeEnum::VoidLake,
                    node_id: Some(0xAB),
                    log_content: "Hello VLF5!".into(),
                    defmt: Some(DefmtLogInfo {
                        location: Some(DefmtLocationInfo {
                            file_path: "main.rs".into(),
                            line_number: "69".into(),
                            module_path: "avionics::core".into(),
                        }),
                        log_level: Level::Debug,
                        timestamp: Some(1.5),
                    }),
                })
                .ok();
            logs_tx
                .send(TargetLog {
                    node_type: NodeTypeEnum::VoidLake,
                    node_id: Some(0xAB),
                    log_content: "Hello VLF5! no location".into(),
                    defmt: Some(DefmtLogInfo {
                        location: None,
                        log_level: Level::Info,
                        timestamp: Some(2.5),
                    }),
                })
                .ok();
            logs_tx
                .send(TargetLog {
                    node_type: NodeTypeEnum::VoidLake,
                    node_id: Some(0xAB),
                    log_content: "Hello VLF5! no timestamp".into(),
                    defmt: Some(DefmtLogInfo {
                        location: Some(DefmtLocationInfo {
                            file_path: "main.rs".into(),
                            line_number: "69".into(),
                            module_path: "gcm::core".into(),
                        }),
                        log_level: Level::Warn,
                        timestamp: None,
                    }),
                })
                .ok();
            logs_tx
                .send(TargetLog {
                    node_type: NodeTypeEnum::RocketWifi,
                    node_id: Some(0x12),
                    log_content: "Hello Rocket WiFi!".into(),
                    defmt: None,
                })
                .ok();
            logs_tx
                .send(TargetLog {
                    node_type: NodeTypeEnum::RocketWifi,
                    node_id: Some(0x12),
                    log_content: "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.".into(),
                    defmt: None,
                })
                .ok();
            sleep(Duration::from_secs(1)).await;

            if stop_rx.try_recv().is_ok() {
                break;
            }
        }

        Ok(())
    }
}
