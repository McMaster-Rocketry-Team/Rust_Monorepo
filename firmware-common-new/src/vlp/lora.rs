pub use embedded_hal_async::delay::DelayNs;
use lora_phy::{
    LoRa, RxMode,
    mod_params::{PacketStatus, RadioError},
    mod_traits::RadioKind,
};

use super::{
    lora_config::LoraConfig,
    radio::{Radio, RxMode as RadioRxMode},
};

pub struct LoraPhy<'a, LK: RadioKind, DL: DelayNs> {
    lora: &'a mut LoRa<LK, DL>,
    lora_config: LoraConfig,
}

impl<'a, RK: RadioKind, DL: DelayNs> LoraPhy<'a, RK, DL> {
    pub fn new(lora: &'a mut LoRa<RK, DL>, lora_config: LoraConfig) -> Self {
        Self { lora, lora_config }
    }
}

impl<'a, RK: RadioKind, DL: DelayNs> Radio for LoraPhy<'a, RK, DL> {
    async fn tx(&mut self, buffer: &[u8]) -> Result<(), RadioError> {
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

    async fn rx(
        &mut self,
        buffer: &mut [u8],
        rx_mode: RadioRxMode,
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

        let listen_mode = match rx_mode {
            RadioRxMode::Single { timeout_ms } => {
                let timeout_us = timeout_ms as u32 * 1_000;
                let symbol_time_us = self.lora_config.symbol_time_us();
                let timeout_symbols = (timeout_us / symbol_time_us) as u16;
                RxMode::Single(timeout_symbols.min(254))
            }
            RadioRxMode::Continuous => RxMode::Continuous,
        };

        self.lora
            .prepare_for_rx(listen_mode, &modulation_params, &rx_pkt_params)
            .await
            .unwrap();
        let (len, status) = self.lora.rx(&rx_pkt_params, buffer).await?;
        Ok((len as usize, status))
    }
}
