use core::future::Future;

use lora_phy::mod_params::{PacketStatus, RadioError};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RxMode {
    Single { timeout_ms: u32 },
    Continuous,
}

pub trait Radio {
    fn tx(&mut self, buffer: &[u8]) -> impl Future<Output = Result<(), RadioError>>;

    fn rx(
        &mut self,
        buffer: &mut [u8],
        mode: RxMode,
    ) -> impl Future<Output = Result<(usize, PacketStatus), RadioError>>;

    fn tx_then_rx(
        &mut self,
        buffer: &mut [u8],
        tx_len: usize,
        rx_mode: RxMode,
    ) -> impl Future<Output = Result<(usize, PacketStatus), RadioError>> {
        async move {
            self.tx(&buffer[..tx_len]).await?;
            self.rx(buffer, rx_mode).await
        }
    }
}
