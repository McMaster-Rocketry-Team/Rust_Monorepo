use firmware_common_new::{rpc::lora_rpc::{LoraRpcClient, LoraRpcRxResult, RxResponse}, vlp::radio::Radio};
use log::error;
use lora_phy::mod_params::{PacketStatus, RadioError};

use crate::gs::serial_wrapper::{Delay, SerialWrapper};

pub struct RpcRadio<'a> {
    client: LoraRpcClient<'a, SerialWrapper, Delay>,
    buffer: [u8; 256],
}

impl<'a> RpcRadio<'a> {
    pub fn new(client: LoraRpcClient<'a, SerialWrapper, Delay>) -> Self {
        Self {
            client,
            buffer: [0u8; 256],
        }
    }
}

impl<'a> Radio for RpcRadio<'a> {
    async fn tx(&mut self, buffer: &[u8]) -> std::result::Result<(), RadioError> {
        self.buffer[..buffer.len()].copy_from_slice(buffer);
        match self.client.tx(buffer.len() as u32, self.buffer).await {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("{:?}", e);
                Err(RadioError::TransmitTimeout)
            }
        }
    }

    async fn rx(
        &mut self,
        buffer: &mut [u8],
        timeout_ms: Option<u16>,
    ) -> std::result::Result<(usize, PacketStatus), RadioError> {
        if let Some(timeout_ms) = timeout_ms {
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
}
