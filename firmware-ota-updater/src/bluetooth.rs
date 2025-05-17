use std::process;

use anyhow::{Result, anyhow};
use btleplug::api::{
    Central as _, Characteristic, Manager as _, Peripheral as _, ScanFilter, ValueNotification,
    WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use futures::stream::StreamExt;
use indicatif::{ProgressBar, ProgressStyle};
use log::{error, info};
use tokio::time::sleep;
use tokio::{
    sync::mpsc,
    time::{Duration, timeout},
};
use uuid::Uuid;

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
///
/// `nodetype` / `nodeid` replicate the optional target-ID feature from Python.
/// Pass `None` to skip.
pub async fn ble_send_file(
    peripheral: &Peripheral,
    data: &[u8],
    nodetype: Option<u8>,
    nodeid: Option<u8>,
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

    // 5) Optional node-selection.
    if let Some(t) = nodetype {
        let v = format!("t{t}");
        peripheral
            .write(&target_char, v.as_bytes(), WriteType::WithoutResponse)
            .await?;
    }
    if let Some(id) = nodeid {
        let v = format!("t{id}");
        peripheral
            .write(&target_char, v.as_bytes(), WriteType::WithoutResponse)
            .await?;
    }

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
    let bar = ProgressBar::new(data.len() as u64);
    bar.set_style(
        ProgressStyle::with_template("{msg} {bar:40.cyan/blue} {bytes}/{total_bytes} ({eta})")?
            .progress_chars("=> "),
    );
    bar.set_message("Uploading");

    // 9) Main transfer loop with sliding-window flow-control.
    let mut sent = 0usize;
    let mut in_window = 0usize;

    for chunk in data.chunks(chunk_size) {
        peripheral
            .write(&chunk_char, chunk, WriteType::WithoutResponse)
            .await?;
        sent += chunk.len();
        in_window += 1;
        bar.set_position(sent as u64);

        if in_window == window && sent < data.len() {
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
    println!("{}", explain_status(final_stat));

    if final_stat == 0 {
        Ok(())
    } else {
        Err(anyhow!("upload failed"))
    }
}

/// Find an adapter, scan for the device, connect, and return an *open* Peripheral.
/// The caller owns the connection; keep the `Peripheral` around between uploads
/// if you want to reuse the link.
///
/// If you know a MAC address or a specific name, filter on that instead of the
/// service UUID – the rest is identical.
pub async fn ble_find_peripheral() -> Result<Peripheral> {
    // 1) Pick the first Bluetooth adapter we have
    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("no Bluetooth adapter found"))?;

    // 2) Start a passive scan.  Most stacks need a couple of seconds.
    adapter.start_scan(ScanFilter::default()).await?;
    info!("Searching for ESP.....");

    let mut count = 0;
    loop {
        let peripherals = adapter.peripherals().await?;
        for peripheral in peripherals {
            let properties = peripheral.properties().await;
            // info!("{:?} {:?}", peripheral, properties);
            if let Ok(Some(properties)) = properties {
                if properties.local_name == Some("RocketOTA".into()) {
                    // 3) Establish a GATT connection
                    peripheral.connect().await?;
                    peripheral.discover_services().await?; // mandatory before I/O
                    return Ok(peripheral);
                }
            }
        }

        count+=1;
        if count > 30 {
            error!("ESP not found");
            process::exit(1);
        }
        sleep(Duration::from_secs(1)).await;
    }
}

/// Gracefully drop the connection.
/// After `.disconnect().await` the hardware link is closed; when the
/// `Peripheral` goes out of scope its handle is cleaned up automatically.
///
/// Calling `disconnect` is *polite*; relying on `Drop` alone still works,
/// but you’ll leave the ESP in the “connected-but-no-master” state for
/// a few seconds.
pub async fn ble_dispose(peripheral: Peripheral) -> Result<()> {
    if peripheral.is_connected().await? {
        peripheral.disconnect().await?;
    }
    Ok(()) // `Peripheral` is consumed and dropped here
}
