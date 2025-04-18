use crate::{
    common::{device_config::LoraConfig, unix_clock::UnixClock}, driver::{clock::Clock, delay::Delay},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use heapless::Vec;
use lora_phy::{
    mod_params::{PacketStatus, RadioError},
    mod_traits::RadioKind,
    LoRa, RxMode,
};

use super::{
    lora_phy::LoraPhy,
    packet::{AckPacket, VLPDownlinkPacket, VLPUplinkPacket},
    packet_builder::{VLPPacketBuilder, MAX_VLP_PACKET_SIZE},
};

// VLP client running on the GCM
pub struct VLPDownlinkClient {
    tx_signal: Signal<NoopRawMutex, VLPUplinkPacket>,
    rx_signal: Signal<NoopRawMutex, (VLPDownlinkPacket, PacketStatus)>,
    send_success_signal: Signal<NoopRawMutex, Option<PacketStatus>>,
}

impl VLPDownlinkClient {
    pub fn new() -> Self {
        VLPDownlinkClient {
            tx_signal: Signal::new(),
            rx_signal: Signal::new(),
            send_success_signal: Signal::new(),
        }
    }

    /// Calling send multiple times concurrently is not supported
    /// Returns the packet status of the ack message
    pub async fn send(&self, packet: VLPUplinkPacket) -> Option<PacketStatus> {
        self.tx_signal.signal(packet);
        self.send_success_signal.wait().await
    }

    pub async fn wait_receive(&self) -> (VLPDownlinkPacket, PacketStatus) {
        self.rx_signal.wait().await
    }

    pub async fn run<'a>(
        &self,
        delay: impl Delay,
        lora: &mut LoRa<impl RadioKind, impl Delay>,
        lora_config: &LoraConfig,
        unix_clock: UnixClock<'a, impl Clock>,
        key: &[u8; 32],
    ) {
        let mut packet_builder = VLPPacketBuilder::new(unix_clock.clone(), lora_config.into(), key);
        let mut lora = LoraPhy::new(lora, lora_config);
        let mut buffer = Vec::<u8, MAX_VLP_PACKET_SIZE>::new();

        loop {
            let result: Result<(), RadioError> = try {
                match lora.rx(RxMode::Single(1000), &mut buffer).await {
                    Ok(packet_status) => {
                        // try to deserialize the packet
                        match packet_builder.deserialize_downlink(&buffer) {
                            Ok(packet) => {
                                self.rx_signal.signal((packet, packet_status));
                            }
                            Err(_) => {
                                // deserialize error
                            }
                        }
                    }
                    Err(RadioError::ReceiveTimeout) => {}
                    Err(e) => Err(e)?,
                }

                if self.tx_signal.signaled() {
                    let tx_packet = self.tx_signal.wait().await;
                    log_info!("Sending message: {:?}", tx_packet);

                    let mut ack_packet_status: Option<PacketStatus> = None;
                    for i in 0..5 {
                        packet_builder
                            .serialize_uplink(&mut buffer, &tx_packet)
                            .unwrap();
                        lora.tx(&buffer).await?;

                        match lora.rx(RxMode::Single(100), &mut buffer).await {
                            Ok(packet_status) => {
                                // try to deserialize the packet
                                match packet_builder.deserialize_downlink(&buffer) {
                                    Ok(packet) => {
                                        if matches!(
                                            packet,
                                            VLPDownlinkPacket::AckPacket(AckPacket { .. })
                                        ) {
                                            log_info!(
                                                "Ack received: rssi: {}, snr: {}",
                                                packet_status.rssi,
                                                packet_status.snr
                                            );
                                            ack_packet_status = Some(packet_status);
                                            break;
                                        } else {
                                            log_warn!(
                                                "Expected AckPacket, but received {:?}",
                                                packet
                                            );
                                        }
                                    }
                                    Err(_) => {
                                        // deserialize error
                                    }
                                }
                            }
                            Err(RadioError::ReceiveTimeout) => {}
                            Err(e) => {
                                Err(e)?;
                            }
                        }

                        log_warn!("Ack not received, retrying {}", i + 1);
                    }
                    if ack_packet_status.is_some() {
                        log_info!("Message sent successfully");
                    } else {
                        log_warn!("Failed to send message");
                    }
                    self.send_success_signal.signal(ack_packet_status);
                }
            };
            if let Err(e) = result {
                log_error!("Error in VLP downlink client");
                delay.delay_ms(1000.0).await;
            }
        }
    }
}
