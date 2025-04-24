pub use embedded_hal_async::delay::DelayNs;
use heapless::Vec;
use lora_phy::{
    mod_params::{PacketStatus, RadioError},
    mod_traits::RadioKind,
    LoRa, RxMode,
};

use super::lora_config::LoraConfig;

pub struct LoraPhy<'a, 'b, LK: RadioKind, DL: DelayNs> {
    lora: &'a mut LoRa<LK, DL>,
    lora_config: &'b LoraConfig,
}

impl<'a, 'b, LK: RadioKind, DL: DelayNs> LoraPhy<'a, 'b, LK, DL> {
    pub fn new(lora: &'a mut LoRa<LK, DL>, lora_config: &'b LoraConfig) -> Self {
        LoraPhy { lora, lora_config }
    }

    pub async fn tx(&mut self, buffer: &[u8]) -> Result<(), RadioError> {
        let modulation_params = self.lora.create_modulation_params(
            self.lora_config.sf_phy(),
            self.lora_config.bw_phy(),
            self.lora_config.cr_phy(),
            self.lora_config.frequency,
        )?;
        let mut tx_params =
            self.lora
                .create_tx_packet_params(8, false, false, false, &modulation_params)?;

        self.lora
            .prepare_for_tx(
                &modulation_params,
                &mut tx_params,
                self.lora_config.power,
                buffer,
            )
            .await?;
        self.lora.tx().await?;
        Ok(())
    }

    pub async fn rx(
        &mut self,
        listen_mode: RxMode,
        buffer: &mut [u8],
    ) -> Result<(usize, PacketStatus), RadioError> {
        let modulation_params = self.lora.create_modulation_params(
            self.lora_config.sf_phy(),
            self.lora_config.bw_phy(),
            self.lora_config.cr_phy(),
            self.lora_config.frequency,
        )?;
        let rx_pkt_params = self.lora.create_rx_packet_params(
            8,
            false,
            buffer.len() as u8,
            false,
            false,
            &modulation_params,
        )?;

        self.lora
            .prepare_for_rx(listen_mode, &modulation_params, &rx_pkt_params)
            .await
            .unwrap();
        let (len, status) = self.lora.rx(&rx_pkt_params, buffer).await?;
        Ok((len as usize, status))
    }
}
