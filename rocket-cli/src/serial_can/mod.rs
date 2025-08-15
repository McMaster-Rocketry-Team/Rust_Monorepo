use std::{io::ErrorKind, time::Duration};

use crate::{
    bluetooth::demultiplex_log::LogDemultiplexer,
    connection_method::{ConnectionMethod, ConnectionMethodFactory, ConnectionOption},
    elf_locator::locate_elf_files,
    gs::serial_wrapper::SerialWrapper,
    monitor::{MonitorStatus, target_log::TargetLog},
};
use anyhow::Result;
use async_trait::async_trait;
use firmware_common_new::{
    can_bus::{
        id::CanBusExtendedId,
        messages::LOG_MESSAGE_TYPE,
        receiver::CanBusMultiFrameDecoder,
        telemetry::{log_multiplexer::DecodedLogFrame, message_aggregator::DecodedMessage},
        usb_can_bus_frame::UsbCanBusFrame,
    },
    rpc::half_duplex_serial::HalfDuplexSerial,
};
use log::{info, warn};
use packed_struct::prelude::*;
use serialport::{SerialPortType, UsbPortInfo, available_ports};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time::sleep,
};

struct SerialConnectionMethodFactory {
    port_name: String,
    name: String,
}

#[async_trait(?Send)]
impl ConnectionMethodFactory for SerialConnectionMethodFactory {
    async fn initialize(&mut self) -> Result<Box<dyn ConnectionMethod>> {
        let elf_info_map = locate_elf_files(None)
            .map_err(|e| warn!("{:?}", e))
            .unwrap_or_default();
        let log_demultiplexer = LogDemultiplexer::new(elf_info_map);

        info!("Opening serial port: {}", self.port_name);
        let serial = serialport::new(self.port_name.clone(), 115200)
            .timeout(Duration::from_secs(5))
            .open()
            .unwrap();

        Ok(Box::new(SerialConnectionMethod {
            serial: SerialWrapper::new(serial),
            log_demultiplexer,
            name: self.name.clone(),
        }))
    }
}

pub struct SerialConnectionMethod {
    serial: SerialWrapper,
    log_demultiplexer: LogDemultiplexer,
    name: String,
}

impl SerialConnectionMethod {
    pub async fn list_options() -> Result<Vec<ConnectionOption>> {
        let mut options = vec![];

        for endgame in available_ports().unwrap().into_iter().filter(|port| {
            matches!(
                port.port_type,
                SerialPortType::UsbPort(UsbPortInfo {
                    vid: 0x120a,
                    pid: 0x0006,
                    ..
                })
            )
        }) {
            options.push(ConnectionOption {
                name: format!("The ENDGAME CAN Bus bridge, {}", endgame.port_name),
                factory: Box::new(SerialConnectionMethodFactory {
                    port_name: endgame.port_name,
                    name: "The ENDGAME".to_string(),
                }),
                attach_only: true,
            });
        }

        for icarus in available_ports().unwrap().into_iter().filter(|port| {
            matches!(
                port.port_type,
                SerialPortType::UsbPort(UsbPortInfo {
                    vid: 0x120a,
                    pid: 0x0004,
                    ..
                })
            )
        }) {
            options.push(ConnectionOption {
                name: format!("ICARUS CAN Bus bridge, {}", icarus.port_name),
                factory: Box::new(SerialConnectionMethodFactory {
                    port_name: icarus.port_name,
                    name: "ICARUS".to_string(),
                }),
                attach_only: true,
            });
        }

        Ok(options)
    }
}

#[async_trait(?Send)]
impl ConnectionMethod for SerialConnectionMethod {
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
        self.serial.set_dtr(true)?;
        status_tx.send(MonitorStatus::Normal).unwrap();

        let mut can_decoder = CanBusMultiFrameDecoder::<16>::new();

        let usb_receive_fut = async {
            let mut buffer = [0u8; { UsbCanBusFrame::SERIALIZED_SIZE * 4 }];
            loop {
                let len = match self.serial.read(&mut buffer).await {
                    Ok(len) => len,
                    Err(serialport::Error {
                        kind: serialport::ErrorKind::Io(ErrorKind::TimedOut),
                        ..
                    }) => {
                        break;
                    }
                    Err(e) => {
                        warn!("serial error: {:?}", e);
                        break;
                    }
                };
                let data = &buffer[..len];
                for frame in data.chunks_exact(UsbCanBusFrame::SERIALIZED_SIZE) {
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
