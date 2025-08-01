use std::time::Duration;

use crate::{
    args::NodeTypeEnum,
    connection_method::ConnectionMethod,
    monitor::{
        MonitorStatus,
        target_log::{DefmtLocationInfo, DefmtLogInfo, TargetLog},
    },
};
use anyhow::Result;
use async_trait::async_trait;
use firmware_common_new::can_bus::{
    messages::{
        baro_measurement::BaroMeasurementMessage,
        node_status::{NodeHealth, NodeMode, NodeStatusMessage},
        vl_status::{FlightStage, VLStatusMessage},
    },
    node_types::{OZYS_NODE_TYPE, VOID_LAKE_NODE_TYPE},
    telemetry::message_aggregator::DecodedMessage,
};
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

    async fn download(&mut self) -> Result<()> {
        info!("Downloading.....");
        sleep(Duration::from_secs(1)).await;
        info!("Download done");
        Ok(())
    }

    async fn attach(
        &mut self,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        mut stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        info!("Attaching.....");
        sleep(Duration::from_millis(500)).await;
        status_tx.send(MonitorStatus::Normal)?;

        let mut void_lake_uptime_s = 0u32;
        let mut ozys_uptime_s = 0u32;
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

            messages_tx
                .send(DecodedMessage {
                    node_type: VOID_LAKE_NODE_TYPE,
                    node_id: 0xAB,
                    message: NodeStatusMessage::new_no_custom_status(
                        void_lake_uptime_s,
                        NodeHealth::Healthy,
                        NodeMode::Operational,
                    )
                    .into(),
                    count: 2,
                })
                .ok();

            messages_tx
                .send(DecodedMessage {
                    node_type: VOID_LAKE_NODE_TYPE,
                    node_id: 0xAB,
                    message: VLStatusMessage {
                        flight_stage: FlightStage::Armed,
                        battery_mv: 7420,
                    }
                    .into(),
                    count: 2,
                })
                .ok();

            messages_tx
                .send(DecodedMessage {
                    node_type: VOID_LAKE_NODE_TYPE,
                    node_id: 0xAB,
                    message: BaroMeasurementMessage::new(0, 101325.5, 25.7).into(),
                    count: 2,
                })
                .ok();

            messages_tx
                .send(DecodedMessage {
                    node_type: OZYS_NODE_TYPE,
                    node_id: 0xFAF,
                    message: NodeStatusMessage::new_no_custom_status(
                        ozys_uptime_s,
                        NodeHealth::Healthy,
                        NodeMode::Operational,
                    )
                    .into(),
                    count: 2,
                })
                .ok();

            void_lake_uptime_s += 1;
            ozys_uptime_s += 1;

            sleep(Duration::from_secs(1)).await;

            if stop_rx.try_recv().is_ok() {
                break;
            }
        }

        Ok(())
    }
}
