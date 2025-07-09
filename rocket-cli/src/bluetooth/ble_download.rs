use anyhow::{Result, anyhow};
use btleplug::api::{Characteristic, Peripheral as _, ValueNotification, WriteType};
use btleplug::platform::Peripheral;
use futures::stream::StreamExt;
use indicatif::{ProgressBar, ProgressStyle};
use log::info;
use tokio::{
    sync::mpsc,
    time::{Duration, timeout},
};
use uuid::Uuid;

use crate::args::NodeTypeEnum;


// @startuml
// scale 2

// participant Laptop
// participant PayloadActivationPCB
// participant TargetNode

// note over Laptop,PayloadActivationPCB : Bluetooth
// note over TargetNode,PayloadActivationPCB : CAN Bus

// Laptop->>PayloadActivationPCB: Handshake start (target node)
// PayloadActivationPCB->>Laptop: Window size

// PayloadActivationPCB->>TargetNode: ResetMessage into_bootloader=true
// TargetNode->>TargetNode: enter DFU
// activate TargetNode

// loop
//   Laptop->>PayloadActivationPCB: Chunk (length = MTU x window size)
//   PayloadActivationPCB->>Laptop: Status
  
//   loop
//     PayloadActivationPCB->>TargetNode: DataTransferMessage (max length 32 bytes)
//     TargetNode->>PayloadActivationPCB: AckMessage
//   end
// end

// Laptop->>PayloadActivationPCB: Transfer End

// PayloadActivationPCB->>TargetNode: DataTransferMessage
// TargetNode->>PayloadActivationPCB: AckMessage
// TargetNode->>TargetNode: exit DFU
// deactivate TargetNode

// PayloadActivationPCB->>Laptop: Status


// @enduml

const CHUNK_CHAR_UUID: Uuid = Uuid::from_u128(0xfba7_891b_18cb_4055_ba5d_0e57396c2fcf);
const READY_CHAR_UUID: Uuid = Uuid::from_u128(0x5ff9_e042_eced_4d02_8f82_c99e81df389b);
const TARGET_ID_CHAR_UUID: Uuid = Uuid::from_u128(0x7090_bb12_25a4_46a2_8a6a_0b78b09bfcb0);
const CTRL_CHAR_UUID: Uuid = Uuid::from_u128(0xd42c_5206_03cc_47d0_aab8_773e02f831fc);

/// Result codes copied 1-for-1 from the Python implementation.
fn explain_status(stat: i32) -> &'static str {
    match stat {
        0 => "Upload complete",
        -1 => "Upload failed: CRC error",
        -2 => "Upload failed: ACK timeout",
        -3 => "Upload failed: a node became inactive",
        -4 => "Upload failed: destination node not found",
        -5 => "Upload failed: timeout / out-of-sync",
        _ => "Upload failed: unknown error",
    }
}

/// Send `data` to an already-connected `peripheral`.
///
/// This call **does not** disconnect at the end – the caller decides the lifetime
/// of the BLE link.  
///
/// `chunk_size` is an upper bound; the effective size is limited to *(MTU-3)*  
/// because BLE subtracts three control-bytes from each ATT packet.

pub async fn ble_download(
    firmware_bytes: &[u8],
    node_type: NodeTypeEnum,
    peripheral: &Peripheral,
) -> Result<()> {
    // 1) Ensure we have performed service discovery.
    peripheral.discover_services().await?;

    // 2) Fetch characteristic handles once.
    let find_char = |uuid: Uuid| -> Result<Characteristic> {
        peripheral
            .characteristics()
            .into_iter()
            .find(|c| c.uuid == uuid)
            .ok_or_else(|| anyhow!("characteristic {uuid} not found"))
    };
    let chunk_char = find_char(CHUNK_CHAR_UUID)?;
    let ready_char = find_char(READY_CHAR_UUID)?;
    let target_char = find_char(TARGET_ID_CHAR_UUID)?;
    let ctrl_char = find_char(CTRL_CHAR_UUID)?;

    // 3) Clamp chunk-size to MTU-3.
    let chunk_size = 244;

    // 4) Ask the device how many chunks it can buffer (the “window”).
    let mut window = 64usize;

    let nodetype: u8 = node_type.into();

    peripheral
        .write(
            &target_char,
            format!("t{nodetype}").as_bytes(),
            WriteType::WithoutResponse,
        )
        .await?;

    // 6) Subscribe to READY notifications and forward them through a channel.
    let (tx, mut rx) = mpsc::channel::<i32>(4);
    peripheral.subscribe(&ready_char).await?;
    let mut notif_stream = peripheral.notifications().await?;
    tokio::spawn(async move {
        while let Some(ValueNotification { uuid, value, .. }) = notif_stream.next().await {
            if uuid == READY_CHAR_UUID {
                if value.len() >= 4 {
                    let stat = i32::from_le_bytes(value[..4].try_into().unwrap());
                    let _ = tx.try_send(stat);
                } else {
                    let _ = tx.try_send(1); // “ready” without payload
                }
            }
        }
    });

    // 7) Tell the ESP we are about to start.
    peripheral
        .write(&ctrl_char, b"start", WriteType::WithoutResponse)
        .await?;

    // 8) Progress indicator.
    let bar = ProgressBar::new(firmware_bytes.len() as u64);
    bar.set_style(
        ProgressStyle::with_template("{msg} {bar:40.cyan/blue} {bytes}/{total_bytes} ({eta})")?
            .progress_chars("=> "),
    );
    bar.set_message("Uploading");

    // 9) Main transfer loop with sliding-window flow-control.
    let mut sent = 0usize;
    let mut in_window = 0usize;

    for chunk in firmware_bytes.chunks(chunk_size) {
        peripheral
            .write(&chunk_char, chunk, WriteType::WithoutResponse)
            .await?;
        sent += chunk.len();
        in_window += 1;
        bar.set_position(sent as u64);

        if in_window == window && sent < firmware_bytes.len() {
            // Wait for a READY with 5-second timeout to avoid hanging forever.
            let stat = timeout(Duration::from_secs(5), rx.recv())
                .await
                .map_err(|_| anyhow!("ready timeout"))?
                .unwrap_or(0);

            if stat < 1 {
                bar.finish_and_clear();
                return Err(anyhow!("device reported error: {}", explain_status(stat)));
            }
            window = stat as usize;
            in_window = 0;
        }
    }

    // 10) Tell the firmware we are done & wait for final status.
    peripheral
        .write(&ctrl_char, b"end", WriteType::WithoutResponse)
        .await?;
    let final_stat = timeout(Duration::from_secs(10), rx.recv())
        .await
        .map_err(|_| anyhow!("final ack timeout"))?
        .unwrap_or(0);

    bar.finish_and_clear();
    info!("{}", explain_status(final_stat));

    if final_stat == 0 {
        Ok(())
    } else {
        Err(anyhow!("upload failed"))
    }
}
