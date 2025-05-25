mod args;
mod attach;
mod bluetooth;
mod connect_method;
mod elf_locator;
mod gen_ota_key;
mod log_viewer;
mod probe;
mod testing;

use anyhow::{Result, anyhow};
use args::Cli;
use args::ModeSelect;
use args::TestingModeSelect;
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
use testing::decode_bluetooth_chunk::test_decode_bluetooth_chunk;

#[tokio::main]
async fn main() -> Result<()> {
    let args = Cli::parse();

    if !matches!(
        args.mode,
        ModeSelect::Testing(TestingModeSelect::DecodeBluetoothChunk(_))
    ) {
        env_logger::builder()
            .filter_level(LevelFilter::Info)
            .try_init()
            .ok();
    }

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
        ModeSelect::Testing(TestingModeSelect::DecodeBluetoothChunk(args)) => {
            test_decode_bluetooth_chunk(args).map_err(|_| anyhow!("Invalid message"))
        }
    }
}
