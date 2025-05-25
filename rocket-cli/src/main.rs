mod attach;
mod bluetooth;
mod connect_method;
mod elf_locator;
mod gen_ota_key;
mod log_viewer;
mod probe;
mod args;

use anyhow::Result;
use args::Cli;
use args::ModeSelect;
use attach::attach_target;
use bluetooth::ble_download::ble_download;
use bluetooth::extract_bin::check_objcopy_installed;
use bluetooth::find_esp::ble_dispose;
use clap::Parser;
use connect_method::ConnectMethod;
use gen_ota_key::gen_ota_key;
use log::LevelFilter;
use probe::probe_download::check_probe_rs_installed;
use probe::probe_download::probe_download;

#[tokio::main]
async fn main() -> Result<()> {
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Info)
        .try_init();
    let args = Cli::parse();

    match args.mode {
        ModeSelect::Download(args) => {
            let connect_method = ConnectMethod::new(&args).await?;
            match &connect_method {
                ConnectMethod::Probe(probe_string) => {
                    check_probe_rs_installed()?;
                    probe_download(&args, probe_string).await?;
                }
                ConnectMethod::OTA(esp) => {
                    check_objcopy_installed()?;
                    ble_download(&args, esp).await?;
                }
            }

            attach_target(&args, &connect_method).await?;

            if let ConnectMethod::OTA(esp) = connect_method {
                ble_dispose(esp).await?;
            }
            Ok(())
        }
        ModeSelect::Attach(args) => {
            let connect_method = ConnectMethod::new(&args).await?;
            match &connect_method {
                ConnectMethod::Probe(_) => {
                    check_probe_rs_installed()?;
                }
                ConnectMethod::OTA(_) => {
                    check_objcopy_installed()?;
                }
            }

            attach_target(&args, &connect_method).await?;

            if let ConnectMethod::OTA(esp) = connect_method {
                ble_dispose(esp).await?;
            }
            Ok(())
        }
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
    }
}
