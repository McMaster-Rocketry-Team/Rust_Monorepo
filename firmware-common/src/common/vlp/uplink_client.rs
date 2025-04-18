use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use heapless::Vec;
use lora_phy::{
    mod_params::{DutyCycleParams, PacketStatus, RadioError},
    mod_traits::RadioKind,
    LoRa, RxMode,
};

use crate::{
    common::{device_config::LoraConfig, unix_clock::UnixClock}, driver::{clock::Clock, delay::Delay},
};

use super::{
    lora_phy::LoraPhy,
    packet::{AckPacket, LowPowerModePacket, VLPDownlinkPacket, VLPUplinkPacket},
    packet_builder::{VLPPacketBuilder, MAX_VLP_PACKET_SIZE},
};

// VLP client running on the rocket
pub struct VLPUplinkClient {
    tx_signal: Signal<NoopRawMutex, VLPDownlinkPacket>,
    rx_signal: Signal<NoopRawMutex, (VLPUplinkPacket, PacketStatus)>,
}

impl VLPUplinkClient {
    pub fn new() -> Self {
        VLPUplinkClient {
            tx_signal: Signal::new(),
            rx_signal: Signal::new(),
        }
    }

    pub fn send(&self, packet: VLPDownlinkPacket) {
        self.tx_signal.signal(packet);
    }

    pub async fn wait_receive(&self) -> (VLPUplinkPacket, PacketStatus) {
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
        let mut low_power_mode = false;

        loop {
            let result: Result<(), RadioError> = try {
                if !low_power_mode {
                    let tx_packet = self.tx_signal.wait().await;
                    packet_builder
                        .serialize_downlink(&mut buffer, &tx_packet)
                        .unwrap();
                    lora.tx(&buffer).await?;
                }

                let listen_mode = if low_power_mode {
                    RxMode::DutyCycle(DutyCycleParams {
                        rx_time: 10_000,     // 10ms
                        sleep_time: 100_000, // 100ms
                    })
                } else {
                    RxMode::Single(100)
                };

                match lora.rx(listen_mode, &mut buffer).await {
                    Ok(packet_status) => {
                        // try to deserialize the packet
                        match packet_builder.deserialize_uplink(&buffer) {
                            Ok(packet) => {
                                if let VLPUplinkPacket::LowPowerModePacket(LowPowerModePacket {
                                    enabled,
                                    ..
                                }) = &packet
                                {
                                    low_power_mode = *enabled;
                                }

                                packet_builder
                                    .serialize_downlink(
                                        &mut buffer,
                                        &AckPacket {
                                            timestamp: unix_clock.now_ms(),
                                        }
                                        .into(),
                                    )
                                    .unwrap();
                                lora.tx(&buffer).await?;

                                self.rx_signal.signal((packet, packet_status));
                            }
                            Err(_) => {
                                // deserialize error
                            }
                        }
                    }
                    Err(RadioError::ReceiveTimeout) => {
                        continue;
                    }
                    Err(e) => Err(e)?,
                };
            };
            if let Err(e) = result {
                log_error!("Error in VLP uplink client");
                delay.delay_ms(1000.0).await;
            }
        }
    }
}
