mod args;
mod bluetooth;
mod connect_method;
mod elf_locator;
mod gen_ota_key;
mod monitor;
mod probe;
mod testing;

use anyhow::{Result, anyhow};
use args::Cli;
use args::ModeSelect;
use args::TestingModeSelect;
use clap::Parser;
use connect_method::get_connection_method;
use gen_ota_key::gen_ota_key;
use log::LevelFilter;
use monitor::monitor_tui;
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
            let mut connection_method =
                get_connection_method(args.force_ota, args.force_probe).await?;

            connection_method
                .download(
                    &args.chip,
                    &args.secret_path,
                    &args.node_type,
                    &args.firmware_elf_path,
                )
                .await?;
            monitor_tui(
                &mut connection_method,
                &args.chip,
                &args.secret_path,
                &args.node_type,
                &args.firmware_elf_path,
            )
            .await?;

            connection_method.dispose().await?;
            Ok(())
        }
        ModeSelect::Attach(args) => {
            let mut connection_method =
                get_connection_method(args.force_ota, args.force_probe).await?;

            monitor_tui(
                &mut connection_method,
                &args.chip,
                &args.secret_path,
                &args.node_type,
                &args.firmware_elf_path,
            )
            .await?;

            connection_method.dispose().await?;
            Ok(())
        }
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
        ModeSelect::Testing(TestingModeSelect::DecodeBluetoothChunk(args)) => {
            test_decode_bluetooth_chunk(args).map_err(|_| anyhow!("Invalid message"))
        }
    }
}
