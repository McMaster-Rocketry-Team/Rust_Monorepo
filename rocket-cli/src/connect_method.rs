use crate::{DownloadCli, bluetooth::find_esp::find_esp, probe::select_probe::select_probe};
use anyhow::{Result, bail};
use btleplug::platform::Peripheral;
use log::info;
use probe_rs::probe::list::Lister;
pub enum ConnectMethod {
    Probe(String),
    OTA(Peripheral),
}

impl ConnectMethod {
    pub async fn new(args: &DownloadCli) -> Result<Self> {
        if args.force_ota && args.force_probe {
            bail!("--force-ota and --force-probe can not be set at the same time")
        }

        let lister = Lister::new();
        let probes = lister.list_all();
        let use_probe = if args.force_probe {
            if probes.len() == 0 {
                bail!("--force-probe is selected but no probe is connected")
            } else {
                true
            }
        } else if args.force_ota {
            false
        } else {
            probes.len() > 0
        };

        if use_probe {
            info!("Using debug probe");
            let probe_string = select_probe(probes).await?;
            Ok(Self::Probe(probe_string))
        } else {
            info!("Using OTA");
            let esp = find_esp().await?;
            Ok(Self::OTA(esp))
        }
    }
}
