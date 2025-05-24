use std::time::Duration;

use anyhow::{Result, bail};
use btleplug::api::{Central as _, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::{Manager, Peripheral};
use log::info;
use tokio::time::sleep;

pub async fn find_esp() -> Result<Peripheral> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = if adapters.len() == 0 {
        bail!("No bluetooth adapter found")
    } else if adapters.len() == 1 {
        info!("Found 1 bluetooth adapter");
        adapters[0].clone()
    } else {
        info!(
            "Found {} bluetooth adapters, using the first one",
            adapters.len()
        );
        adapters[0].clone()
    };

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
                    peripheral.connect().await?;
                    peripheral.discover_services().await?;
                    return Ok(peripheral);
                }
            }
        }

        count += 1;
        if count > 30 {
            bail!("ESP not found");
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
