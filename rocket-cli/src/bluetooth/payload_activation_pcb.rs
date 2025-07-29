use std::time::Duration;

use anyhow::Ok;
use anyhow::Result;
use anyhow::anyhow;
use btleplug::api::ValueNotification;
use btleplug::api::WriteType;
use btleplug::{
    api::{Characteristic, Peripheral as _},
    platform::Peripheral,
};
use futures::StreamExt;
use log::info;
use log::warn;
use tokio::runtime::Handle;
use tokio::sync::mpsc;
use tokio::time::timeout;
use uuid::Uuid;

use crate::args::NodeTypeEnum;

pub struct PayloadActivationPCB {
    peripheral: Peripheral,
    chunk_char: Characteristic,
    ctrl_char: Characteristic,
    status_rx: mpsc::Receiver<u8>,
    pub log_rx: mpsc::Receiver<Vec<u8>>,
}

impl PayloadActivationPCB {
    pub async fn new(peripheral: Peripheral) -> Result<Self> {
        peripheral.discover_services().await?;

        let chars = peripheral.characteristics();
        let get_char = |uuid: u128| {
            let uuid = Uuid::from_u128(uuid);
            chars
                .iter()
                .find(|c| c.uuid == uuid)
                .cloned()
                .ok_or_else(|| anyhow!("characteristic {uuid} not found"))
        };

        let chunk_char = get_char(0xfba7891b18cb4055ba5d0e57396c2fcf)?;
        let status_char = get_char(0x5ff9e042eced4d028f82c99e81df389b)?;
        let ctrl_char = get_char(0xd42c520603cc47d0aab8773e02f831fc)?;
        let log_char = get_char(0xd301e88042164ba78de7159bdda2f1ac)?;

        peripheral.subscribe(&status_char).await?;
        peripheral.subscribe(&log_char).await?;
        let mut notif_stream = peripheral.notifications().await?;

        let (status_tx, status_rx) = mpsc::channel::<u8>(2);
        let (log_tx, log_rx) = mpsc::channel::<Vec<u8>>(10);
        tokio::spawn(async move {
            while let Some(ValueNotification { uuid, value }) = notif_stream.next().await {
                if uuid == status_char.uuid {
                    status_tx.try_send(value[0]).ok();
                } else if uuid == log_char.uuid {
                    info!("received log char");
                    let success = log_tx.try_send(value).is_ok();
                    if !success {
                        warn!("log_tx overflow");
                    }
                }
            }
        });

        Ok(Self {
            peripheral,
            chunk_char,
            ctrl_char,
            status_rx,
            log_rx,
        })
    }

    async fn send_ctrl_no_reply(&mut self, cmd: &[u8]) -> Result<()> {
        self.peripheral
            .write(&self.ctrl_char, cmd, WriteType::WithoutResponse)
            .await?;

        Ok(())
    }

    async fn send_ctrl(&mut self, cmd: &[u8]) -> Result<()> {
        self.send_ctrl_no_reply(cmd).await?;

        let status = timeout(Duration::from_secs(5), self.status_rx.recv())
            .await
            .map_err(|_| anyhow!("status timeout"))?
            .ok_or(anyhow!("status_rx channel closed"))?;

        match status {
            0 => Ok(()),
            1 => Err(anyhow!("CrcError")),
            2 => Err(anyhow!("AckTimeout")),
            3 => Err(anyhow!("NodeDropped")),
            4 => Err(anyhow!("NodeTypeEmpty")),
            5 => Err(anyhow!("ErrorBleTimeout")),
            6 => Err(anyhow!("ErrorBleDisconnected")),
            7 => Err(anyhow!("ErrorBleLostSync")),
            8 => Err(anyhow!("ErrorBleWrongChunkSequence")),
            9 => Err(anyhow!("ErrorBleWrongChunkLength")),
            10 => Err(anyhow!("ErrorSelfOta")),
            11 => Err(anyhow!("ErrorInternal")),
            _ => Err(anyhow!("unknown error")),
        }
    }

    pub async fn ota(&mut self, firmware_bytes: &[u8], node_type: NodeTypeEnum) -> Result<()> {
        static CHUNK_SIZE: usize = 244;
        static WINDOW_SIZE: usize = 32;

        info!("initializing ota.....");
        // start session with node type
        self.send_ctrl(&[0x2u8, node_type.into(), 0]).await?;

        info!("uploading firmware.....");
        let mut seq_num = 0u16;
        let mut sent = 0usize;
        for window in firmware_bytes.chunks(CHUNK_SIZE * WINDOW_SIZE) {
            // window start
            let mut buf = [0x8, 0, 0];
            buf[1..3].copy_from_slice(&seq_num.to_le_bytes());
            self.send_ctrl_no_reply(&buf).await?;

            for chunk in window.chunks(CHUNK_SIZE) {
                self.peripheral
                    .write(&self.chunk_char, chunk, WriteType::WithoutResponse)
                    .await?;
                sent += chunk.len();
            }

            seq_num += 1;
            // window end
            let mut buf = [0x10, 0, 0];
            if sent == firmware_bytes.len() {
                buf[0] |= 0x20;
            }
            buf[1..3].copy_from_slice(&(window.len() as u16).to_le_bytes());
            self.send_ctrl(&buf).await?;

            info!(
                "uploaded {}/{} ({}%)",
                sent,
                firmware_bytes.len(),
                (sent as f32 / firmware_bytes.len() as f32 * 100f32).round()
            );
        }

        info!("upload success");

        Ok(())
    }
}

impl Drop for PayloadActivationPCB {
    fn drop(&mut self) {
        warn!("try to drop pab");
        // Handle::current().block_on(async {
        //     if self.peripheral.is_connected().await.unwrap_or(false) {
        //         self.peripheral.disconnect().await.ok();
        //     }
        // });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_ota() {
        println!("{:?}", 10u16.to_le_bytes());
    }
}
