mod args;
mod bluetooth;
mod connection_method;
mod elf_locator;
mod gen_key;
mod gs;
mod monitor;
mod probe;
mod serial_can;
mod testing;

use std::fs::File;
use std::io;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use anyhow::Ok;
use anyhow::{Result, anyhow};
use args::Cli;
use args::ModeSelect;
use args::TestingModeSelect;
use chrono::Local;
use clap::Parser;
use connection_method::ConnectionMethod;
use connection_method::get_connection_method;
use fern::Dispatch;
use fern::colors::Color;
use fern::colors::ColoredLevelConfig;
use gen_key::gen_ota_key;
use log::LevelFilter;
use monitor::monitor_tui;
use testing::decode_bluetooth_chunk::test_decode_bluetooth_chunk;
use testing::mock_connection_method::MockConnectionMethod;

use crate::connection_method::get_esp_connection_method;
use crate::gen_key::gen_vlp_key;
use crate::gs::find_ground_station::find_ground_station;
use crate::gs::ground_station_tui;
use crate::testing::mock_ground_station::mock_ground_station_tui;
use crate::testing::send_fake_vlp_telemetry::send_fake_vlp_telemetry;

#[tokio::main]
async fn main() -> Result<()> {
    init_logging()?;

    let args = Cli::parse();
    match args.mode {
        ModeSelect::Download(args) => {
            let mut connection_method = get_connection_method(
                true,
                Some(args.chip),
                Some(args.firmware_elf_path.clone()),
                Some(args.node_type),
                Some(args.secret_path),
            )
            .await?;

            connection_method.download().await?;
            monitor_tui(&mut connection_method, Some(&args.firmware_elf_path)).await
        }
        ModeSelect::DownloadEsp(args) => {
            let mut connection_method =
                get_esp_connection_method(args.firmware_bin_path, args.node_type, args.secret_path)
                    .await?;

            connection_method.download().await
        }
        ModeSelect::Attach(args) => {
            let mut connection_method =
                get_connection_method(false, args.chip, args.elf, None, None).await?;

            monitor_tui(&mut connection_method, None).await
        }
        ModeSelect::GroundStation => {
            let serial_path = find_ground_station().await?;
            ground_station_tui(&serial_path).await
        }
        ModeSelect::GenVlpKey(args) => gen_vlp_key(args),
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
        ModeSelect::Testing(TestingModeSelect::DecodeBluetoothChunk(args)) => {
            test_decode_bluetooth_chunk(args).map_err(|e| anyhow!("{:?}", e))
        }
        ModeSelect::Testing(TestingModeSelect::MockConnection) => {
            let mut connection_method: Box<dyn ConnectionMethod> = Box::new(MockConnectionMethod);

            monitor_tui(&mut connection_method, None).await
        }
        ModeSelect::Testing(TestingModeSelect::MockGroundStation) => {
            mock_ground_station_tui().await
        }
        ModeSelect::Testing(TestingModeSelect::SendVLPTelemetry(args)) => {
            send_fake_vlp_telemetry(args).await
        }
    }
}

static STDOUT_ENABLED: AtomicBool = AtomicBool::new(true);

pub fn enable_stdout_logging(enabled: bool) {
    STDOUT_ENABLED.store(enabled, Ordering::Relaxed);
}

fn init_logging() -> Result<()> {
    let colors = ColoredLevelConfig::new()
        .error(Color::Red)
        .warn(Color::Yellow)
        .info(Color::Green)
        .debug(Color::Blue)
        .trace(Color::Magenta);

    let stdout = Dispatch::new()
        .filter(|_| STDOUT_ENABLED.load(Ordering::Relaxed))
        .level(LevelFilter::Info)
        .level_for("rocket_cli", LevelFilter::Trace)
        .level_for("firmware_common_new", LevelFilter::Trace)
        .format(move |out, message, record| {
            out.finish(format_args!(
                "{lvl:<5}[{target}] {msg}",
                lvl = colors.color(record.level()),
                target = record.target(),
                msg = message
            ))
        })
        .chain(io::stdout());

    let logfile = Dispatch::new()
        .level(LevelFilter::Info)
        .level_for("rocket_cli", LevelFilter::Trace)
        .level_for("firmware_common_new", LevelFilter::Trace)
        .format(|out, message, record| {
            out.finish(format_args!(
                "{} {:<5} [{}] {}",
                Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.target(),
                message
            ))
        })
        .chain(File::create(".rocket-cli.log")?);

    Dispatch::new().chain(stdout).chain(logfile).apply()?;

    Ok(())
}
