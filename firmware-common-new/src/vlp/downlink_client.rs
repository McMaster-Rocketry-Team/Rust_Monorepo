use super::{
    lora_config::LoraConfig,
    lora_phy::LoraPhy,
    packets::{VLPDownlinkPacket, VLPUplinkPacket, MAX_VLP_PACKET_SIZE},
};
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::RawMutex, signal::Signal};
use embedded_hal_async::delay::DelayNs;
use lora_phy::{
    mod_params::{PacketStatus, RadioError},
    mod_traits::RadioKind,
    LoRa, RxMode,
};
use sha2::Digest;
use sha2::Sha256;

// VLP client running on the GCM
pub struct VLPDownlinkClient<M: RawMutex> {
    tx_signal: Signal<M, VLPUplinkPacket>,
    tx_result_signal: Signal<M, Result<PacketStatus, VLPTXError>>,
    rx_signal: Signal<M, (VLPDownlinkPacket, PacketStatus)>,
}

impl<M: RawMutex> VLPDownlinkClient<M> {
    pub fn new() -> Self {
        VLPDownlinkClient {
            tx_signal: Signal::new(),
            rx_signal: Signal::new(),
            tx_result_signal: Signal::new(),
        }
    }

    /// Calling send multiple times concurrently is not supported
    /// Returns the packet status of the ack message
    pub async fn send(&self, packet: VLPUplinkPacket) -> Result<PacketStatus, VLPTXError> {
        self.tx_signal.signal(packet);
        self.tx_result_signal.wait().await
    }

    pub async fn wait_receive(&self) -> (VLPDownlinkPacket, PacketStatus) {
        self.rx_signal.wait().await
    }

    pub async fn run(
        &self,
        lora: &mut LoRa<impl RadioKind, impl DelayNs>,
        lora_config: &LoraConfig,
        key: &[u8; 32],
    ) {
        let mut lora = LoraPhy::new(lora, lora_config);
        let mut buffer = [0u8; MAX_VLP_PACKET_SIZE]; 
        loop {
            if let Err(e) = self.rx_tx_cycle(&mut buffer,&mut lora, key).await {
                log_warn!("Error in VLP rx_tx_cycle: {:?}", e);
                yield_now().await;
            }
        }
    }

    async fn rx_tx_cycle<'a, 'b>(
        &self,
        buffer: &mut [u8],
        lora: &mut LoraPhy<'a, 'b, impl RadioKind, impl DelayNs>,
        key: &[u8; 32],
    ) -> Result<(), RxTxCycleError> {
        let (rx_len, packet_status) = lora
            .rx(RxMode::Continuous, buffer)
            .await
            .map_err(RxTxCycleError::Radio)?;
        let rx_packet = VLPDownlinkPacket::deserialize(&buffer[..rx_len])
            .ok_or(RxTxCycleError::DeserializeError)?;
        self.rx_signal.signal((rx_packet, packet_status));

        if let Some(tx_packet) = self.tx_signal.try_take() {
            self.tx_result_signal
                .signal(self.tx(tx_packet, buffer, rx_len, lora, key).await);
        }

        Ok(())
    }

    async fn tx<'a, 'b>(
        &self,
        tx_packet: VLPUplinkPacket,
        buffer: &mut [u8],
        rx_len: usize,
        lora: &mut LoraPhy<'a, 'b, impl RadioKind, impl DelayNs>,
        key: &[u8; 32],
    ) -> Result<PacketStatus, VLPTXError> {
        let mut hasher = Sha256::new();
        hasher.update(key);
        hasher.update(&buffer[..rx_len]);

        // packet
        let mut offset = tx_packet.serialize(buffer);

        // sha256 signed
        hasher.update(&buffer[..offset]);
        let hash = hasher.finalize();
        buffer[offset..(offset + 16)].copy_from_slice(&hash[0..16]);
        offset += 16;
        let expected_ack_sha = u16::from_be_bytes([hash[0], hash[1]]);

        // ecc
        let ecc_len = offset / 4;
        let encoder = reed_solomon::Encoder::new(ecc_len);
        let encoded = encoder.encode(&buffer[..offset]);
        buffer[offset..(offset + ecc_len)].copy_from_slice(&encoded.ecc());
        offset += ecc_len;

        // send the packet
        lora.tx(&buffer[..offset])
            .await
            .map_err(VLPTXError::Radio)?;

        match lora.rx(RxMode::Single(100), buffer).await {
            Ok((rx_len, packet_status)) => {
                if let Some(VLPDownlinkPacket::Ack(ack_packet)) =
                    VLPDownlinkPacket::deserialize(&buffer[..rx_len])
                    && ack_packet.crc == expected_ack_sha
                {
                    return Ok(packet_status);
                } else {
                    return Err(VLPTXError::WrongAck);
                }
            }
            Err(RadioError::ReceiveTimeout) => {
                return Err(VLPTXError::AckNotReceived);
            }
            Err(e) => {
                return Err(VLPTXError::Radio(e));
            },
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
enum RxTxCycleError {
    Radio(RadioError),
    DeserializeError,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum VLPTXError {
    Radio(RadioError),
    AckNotReceived,
    WrongAck,
}
