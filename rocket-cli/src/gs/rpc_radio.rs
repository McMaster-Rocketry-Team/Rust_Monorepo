use firmware_common_new::{
    rpc::lora_rpc::{LoraRpcClient, LoraRpcRxResult, RxResponse, TxThenRxResponse},
    vlp::radio::{Radio, RxMode},
};
use log::error;
use lora_phy::mod_params::{PacketStatus, RadioError};

use crate::gs::serial_wrapper::SerialWrapper;

pub struct RpcRadio<'a> {
    client: LoraRpcClient<'a, SerialWrapper>,
    buffer: [u8; 256],
    after_tx: Option<Box<dyn FnOnce()>>,
}

impl<'a> RpcRadio<'a> {
    pub fn new(
        client: LoraRpcClient<'a, SerialWrapper>,
        after_tx: Option<Box<dyn FnOnce()>>,
    ) -> Self {
        Self {
            client,
            buffer: [0u8; 256],
            after_tx,
        }
    }
}

impl<'a> Radio for RpcRadio<'a> {
    async fn tx(&mut self, buffer: &[u8]) -> std::result::Result<(), RadioError> {
        self.buffer[..buffer.len()].copy_from_slice(buffer);
        let result = match self.client.tx(buffer.len() as u32, self.buffer).await {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("{:?}", e);
                Err(RadioError::TransmitTimeout)
            }
        };

        if let Some(after_tx) = self.after_tx.take() {
            after_tx();
        }

        result
    }

    async fn rx(
        &mut self,
        buffer: &mut [u8],
        mode: RxMode,
    ) -> std::result::Result<(usize, PacketStatus), RadioError> {
        if let RxMode::Single { timeout_ms } = mode {
            match self.client.rx(timeout_ms as u32).await {
                Ok(RxResponse {
                    result:
                        LoraRpcRxResult::Success {
                            len,
                            data,
                            rssi,
                            snr,
                        },
                }) => {
                    buffer[..(len as usize)].copy_from_slice(&data[..(len as usize)]);

                    Ok((len as usize, PacketStatus { rssi, snr }))
                }
                Ok(RxResponse {
                    result: LoraRpcRxResult::Timeout,
                }) => Err(RadioError::ReceiveTimeout),
                Ok(RxResponse {
                    result: LoraRpcRxResult::Error,
                }) => {
                    error!("rx failed, unknown reason");
                    Err(RadioError::Reset)
                }
                Err(e) => {
                    error!("rx rpc communication failed: {:?}", e);
                    Err(RadioError::Reset)
                }
            }
        } else {
            loop {
                match self.client.rx(4000).await {
                    Ok(RxResponse {
                        result:
                            LoraRpcRxResult::Success {
                                len,
                                data,
                                rssi,
                                snr,
                            },
                    }) => {
                        buffer[..(len as usize)].copy_from_slice(&data[..(len as usize)]);

                        return Ok((len as usize, PacketStatus { rssi, snr }));
                    }
                    Ok(RxResponse {
                        result: LoraRpcRxResult::Timeout,
                    }) => {
                        continue;
                    }
                    Ok(RxResponse {
                        result: LoraRpcRxResult::Error,
                    }) => {
                        error!("rx failed, unknown reason");
                        return Err(RadioError::Reset);
                    }
                    Err(e) => {
                        error!("rx rpc communication failed: {:?}", e);
                        return Err(RadioError::Reset);
                    }
                }
            }
        }
    }

    async fn tx_then_rx(
        &mut self,
        buffer: &mut [u8],
        tx_len: usize,
        rx_mode: RxMode,
    ) -> Result<(usize, PacketStatus), RadioError> {
        let result = if let RxMode::Single { timeout_ms } = rx_mode {
            self.buffer[..tx_len].copy_from_slice(&buffer[..tx_len]);
            match self
                .client
                .tx_then_rx(tx_len as u32, self.buffer, timeout_ms)
                .await
            {
                Ok(TxThenRxResponse {
                    tx_success: true,
                    result:
                        LoraRpcRxResult::Success {
                            len,
                            data,
                            rssi,
                            snr,
                        },
                }) => {
                    buffer[..(len as usize)].copy_from_slice(&data[..(len as usize)]);

                    Ok((len as usize, PacketStatus { rssi, snr }))
                }
                Ok(TxThenRxResponse {
                    tx_success: true,
                    result: LoraRpcRxResult::Timeout,
                }) => Err(RadioError::ReceiveTimeout),
                Ok(TxThenRxResponse {
                    tx_success: true,
                    result: LoraRpcRxResult::Error,
                }) => {
                    error!("rx failed, unknown reason");
                    Err(RadioError::Reset)
                }
                Ok(TxThenRxResponse {
                    tx_success: false, ..
                }) => {
                    error!("tx failed, unknown reason");
                    Err(RadioError::Reset)
                }
                Err(e) => {
                    error!("rx rpc communication failed: {:?}", e);
                    Err(RadioError::Reset)
                }
            }
        } else {
            unimplemented!()
        };

        if let Some(after_tx) = self.after_tx.take() {
            after_tx();
        }

        result
    }
}
