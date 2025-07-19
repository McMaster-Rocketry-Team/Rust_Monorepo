use std::time::Duration;

use crate::{
    bluetooth::demultiplex_log::LogDemultiplexer,
    connection_method::{ConnectionMethod, ConnectionMethodFactory, ConnectionOption},
    elf_locator::locate_elf_files,
    monitor::{MonitorStatus, target_log::TargetLog},
};
use anyhow::Result;
use async_trait::async_trait;
use firmware_common_new::can_bus::{
    id::CanBusExtendedId,
    messages::LOG_MESSAGE_TYPE,
    receiver::CanBusMultiFrameDecoder,
    telemetry::{log_multiplexer::DecodedLogFrame, message_aggregator::DecodedMessage},
    usb_can_bus_frame::UsbCanBusFrame,
};
use log::warn;
use nusb::{DeviceInfo, Interface, transfer::RequestBuffer};
use packed_struct::prelude::*;
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
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
            interface: device.claim_interface(1)?,
            log_demultiplexer,
            name: self.name.clone(),
        }))
    }
}

pub struct USBConnectionMethod {
    interface: Interface,
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
                    "The ENDGAME CAN Bus bridge, SN {}",
                    endgame.serial_number().unwrap_or("N/A")
                ),
                factory: Box::new(USBConnectionMethodFactory {
                    device_info: endgame,
                    name: "The ENDGAME".to_string(),
                }),
                attach_only: true,
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
                attach_only: true,
            });
        }

        Ok(options)
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for USBConnectionMethod {
    fn name(&self) -> String {
        self.name.clone()
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
        status_tx.send(MonitorStatus::Normal).unwrap();

        let mut can_decoder = CanBusMultiFrameDecoder::<16>::new();

        let usb_receive_fut = async {
            loop {
                let data = self
                    .interface
                    .interrupt_in(0x82, RequestBuffer::new(64))
                    .await
                    .into_result();
                let data = match data {
                    Ok(data) => data,
                    Err(e) => {
                        warn!("usb transfer error: {:?}", e);
                        break;
                    }
                };
                let frame_count = data[0] as usize;
                for i in 0..frame_count {
                    let start = 1 + (i * UsbCanBusFrame::SERIALIZED_SIZE);
                    let frame = &data[start..(start + UsbCanBusFrame::SERIALIZED_SIZE)];

                    let frame = UsbCanBusFrame::unpack_from_slice(frame).unwrap();
                    if frame.data_length > 8 {
                        warn!("Received a CAN frame with more than 8 bytes of data, skipping");
                        continue;
                    }

                    let parsed_id = CanBusExtendedId::from_raw(frame.id);
                    if parsed_id.message_type == LOG_MESSAGE_TYPE {
                        let mut data = heapless::Vec::<u8, 8>::new();
                        data.extend_from_slice(frame.data()).unwrap();
                        self.log_demultiplexer.process_frame(
                            DecodedLogFrame {
                                node_type: parsed_id.node_type,
                                node_id: parsed_id.node_id,
                                data,
                            },
                            &logs_tx,
                        );
                    } else {
                        let timestamp = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_millis() as u64;
                        let frame = (timestamp, frame.id, frame.data());
                        if let Some(m) = can_decoder.process_frame(&frame) {
                            messages_tx
                                .send(DecodedMessage {
                                    node_type: m.data.id.node_type,
                                    node_id: m.data.id.node_id,
                                    message: m.data.message,
                                    count: 1,
                                })
                                .ok();
                        }
                    }
                }
            }
        };

        tokio::select! {
            _ = usb_receive_fut => {}
            _ = stop_rx => {}
        }

        Ok(())
    }
}
