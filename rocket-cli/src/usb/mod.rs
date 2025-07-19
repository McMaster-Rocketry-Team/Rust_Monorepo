use std::time::Duration;

use anyhow::Result;
use async_trait::async_trait;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use log::warn;
use nusb::{Device, DeviceInfo};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
};

use crate::{
    bluetooth::demultiplex_log::LogDemultiplexer,
    connection_method::{ConnectionMethod, ConnectionMethodFactory, ConnectionOption},
    elf_locator::locate_elf_files,
    monitor::{MonitorStatus, target_log::TargetLog},
};

struct USBConnectionMethodFactory {
    device_info: DeviceInfo,
    name: String,
}

#[async_trait(?Send)]
impl ConnectionMethodFactory for USBConnectionMethodFactory {
    async fn initialize(&mut self) -> Result<Box<dyn ConnectionMethod>> {
        let elf_info_map = locate_elf_files(None)
            .map_err(|e| warn!("{:?}", e))
            .unwrap_or_default();
        let log_demultiplexer = LogDemultiplexer::new(elf_info_map);

        let device = self.device_info.open()?;
        Ok(Box::new(USBConnectionMethod {
            device,
            log_demultiplexer,
            name: self.name.clone(),
        }))
    }
}

pub struct USBConnectionMethod {
    device: Device,
    log_demultiplexer: LogDemultiplexer,
    name: String,
}

impl USBConnectionMethod {
    pub async fn list_options() -> Result<Vec<ConnectionOption>> {
        let mut options = vec![];

        for endgame in nusb::list_devices()?
            .filter(|device| device.vendor_id() == 0x120a && device.product_id() == 0x0006)
        {
            options.push(ConnectionOption {
                name: format!(
                    "The ENDGAME CAN Bus bridge, SN {} (attach only, no download)",
                    endgame.serial_number().unwrap_or("N/A")
                ),
                factory: Box::new(USBConnectionMethodFactory {
                    device_info: endgame,
                    name: "The ENDGAME".to_string(),
                }),
            });
        }

        for icarus in nusb::list_devices()?
            .filter(|device| device.vendor_id() == 0x120a && device.product_id() == 0x0004)
        {
            options.push(ConnectionOption {
                name: format!(
                    "ICARUS CAN Bus bridge, SN {} (attach only, no download)",
                    icarus.serial_number().unwrap_or("N/A")
                ),
                factory: Box::new(USBConnectionMethodFactory {
                    device_info: icarus,
                    name: "ICARUS".to_string(),
                }),
            });
        }

        Ok(options)
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for USBConnectionMethod {
    fn name(&self) -> String {
        String::from("RocketOTA")
    }

    async fn download(&mut self) -> Result<()> {
        warn!("USB connection method is not configured for download, skipping");
        sleep(Duration::from_secs(1)).await;
        Ok(())
    }

    async fn attach(
        &mut self,
        status_tx: watch::Sender<MonitorStatus>,
        logs_tx: broadcast::Sender<TargetLog>,
        messages_tx: broadcast::Sender<DecodedMessage>,
        stop_rx: oneshot::Receiver<()>,
    ) -> Result<()> {
        todo!()
    }
}
