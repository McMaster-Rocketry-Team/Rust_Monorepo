use std::time::Duration;

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
use tokio::runtime::Handle;
use tokio::sync::mpsc;
use tokio::time::timeout;
use uuid::Uuid;

use crate::args::NodeTypeEnum;

pub struct PayloadActivationPCB {
    peripheral: Peripheral,
    chunk_char: Characteristic,
    target_char: Characteristic,
    ctrl_char: Characteristic,
    ready_rx: mpsc::Receiver<u8>,
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

        let chunk_char = get_char(0xfba7_891b_18cb_4055_ba5d_0e57396c2fcf)?;

        // TODO: remove variable window size in firmware
        let ready_char = get_char(0x5ff9_e042_eced_4d02_8f82_c99e81df389b)?;
        let target_char = get_char(0x7090_bb12_25a4_46a2_8a6a_0b78b09bfcb0)?;
        let ctrl_char = get_char(0xd42c_5206_03cc_47d0_aab8_773e02f831fc)?;

        // TODO: add to firmware
        let log_char = get_char(0xaf91_66c0_96c9_4917_b4ed_709938f3676d)?;

        let mut notif_stream = peripheral.notifications().await?;

        let (ready_tx, ready_rx) = mpsc::channel::<u8>(2);
        let (log_tx, log_rx) = mpsc::channel::<Vec<u8>>(2);
        tokio::spawn(async move {
            while let Some(ValueNotification { uuid, value }) = notif_stream.next().await {
                if uuid == ready_char.uuid {
                    ready_tx.try_send(value[0]).ok();
                } else if uuid == log_char.uuid {
                    log_tx.try_send(value).ok();
                }
            }
        });

        Ok(Self {
            peripheral,
            chunk_char,
            target_char,
            ctrl_char,
            ready_rx,
            log_rx,
        })
    }

    pub async fn ota(&mut self, firmware_bytes: &[u8], node_type: NodeTypeEnum) -> Result<()> {
        static CHUNK_SIZE: usize = 244;
        static WINDOW_SIZE: usize = 64;

        let stat_to_error = |stat: u8| {
            match stat {
                0 => Ok(()),
                1 => Err(anyhow!("upload failed: CRC error")),
                2 => Err(anyhow!("upload failed: ACK timeout")),
                3 => Err(anyhow!("upload failed: a node became inactive")),
                4 => Err(anyhow!("upload failed: destination node not found")),
                5 => Err(anyhow!("upload failed: timeout / out-of-sync")),
                _ => Err(anyhow!("upload failed: unknown error")),
            }
        };

        info!("initializing ota.....");
        let nodetype: u8 = node_type.into();
        self.peripheral
            .write(
                &self.target_char,
                format!("t{nodetype}").as_bytes(),
                WriteType::WithoutResponse,
            )
            .await?;

        self.peripheral
            .write(&self.ctrl_char, b"start", WriteType::WithoutResponse)
            .await?;

        info!("uploading firmware.....");
        let mut i = 0usize;
        let mut sent = 0usize;
        for chunk in firmware_bytes.chunks(CHUNK_SIZE) {
            self.peripheral
                .write(&self.chunk_char, chunk, WriteType::WithoutResponse)
                .await?;
            i += 1;
            sent += chunk.len();

            if i == WINDOW_SIZE && sent < firmware_bytes.len() {
                let stat = timeout(Duration::from_secs(5), self.ready_rx.recv())
                    .await
                    .map_err(|_| anyhow!("ready timeout"))?
                    .ok_or(anyhow!("ready_rx channel closed"))?;
                // TODO: use 0 to indicate no problem
                stat_to_error(stat)?;

                info!(
                    "uploaded {}/{} ({}%)",
                    sent,
                    firmware_bytes.len(),
                    (sent as f32 / firmware_bytes.len() as f32 * 100f32).round()
                );

                i = 0;
            }
        }

        self.peripheral
            .write(&self.ctrl_char, b"end", WriteType::WithoutResponse)
            .await?;
        let stat = timeout(Duration::from_secs(5), self.ready_rx.recv())
            .await
            .map_err(|_| anyhow!("ready timeout"))?
            .ok_or(anyhow!("ready_rx channel closed"))?;
        stat_to_error(stat)?;

        info!("upload success");

        Ok(())
    }
}

impl Drop for PayloadActivationPCB {
    fn drop(&mut self) {
        Handle::current().block_on(async {
            if self.peripheral.is_connected().await.unwrap_or(false) {
                self.peripheral.disconnect().await.ok();
            }
        });
    }
}
