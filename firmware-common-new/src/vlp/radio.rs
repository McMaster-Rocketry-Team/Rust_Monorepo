use core::future::Future;

use lora_phy::mod_params::{PacketStatus, RadioError};

pub trait Radio {
    fn tx(&mut self, buffer: &[u8]) -> impl Future<Output = Result<(), RadioError>>;

    fn rx(
        &mut self,
        buffer: &mut [u8],
        timeout_ms: Option<u16>,
    ) -> impl Future<Output = Result<(usize, PacketStatus), RadioError>>;
}
