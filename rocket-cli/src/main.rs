mod download_probe;
mod gen_ota_key;
mod log_viewer_tui;
mod target_log;

use anyhow::Result;
use anyhow::bail;
use clap::Parser;
use clap::Subcommand;
use download_probe::download_probe;
use gen_ota_key::gen_ota_key;
use log::LevelFilter;
use log::info;
use log_viewer_tui::log_viewer_tui;
use probe_rs::probe::list::Lister;
use tokio::join;
use tokio::sync::broadcast;
use tokio::sync::oneshot;

#[derive(Parser, Debug)]
#[command(name = "Rocket CLI")]
#[command(bin_name = "rocket-cli")]
struct Cli {
    #[clap(subcommand)]
    mode: ModeSelect,
}

#[derive(Subcommand, Debug)]
enum ModeSelect {
    #[command(about = "download firmware to target via probe or ota")]
    Download(DownloadCli),

    #[command(about = "generate private and public keys for ota")]
    GenOtaKey(GenOtaKeyCli),
}

#[derive(Parser, Debug)]
struct DownloadCli {
    #[arg(long, help = "force using ota")]
    force_ota: bool,
    #[arg(long, help = "force using probe")]
    force_probe: bool,
    chip: String,
    secret_path: std::path::PathBuf,
    node_type: NodeTypeEnum,
    firmware_elf_path: std::path::PathBuf,
}

#[derive(clap::ValueEnum, Clone, Debug)]
enum NodeTypeEnum {
    VoidLake,
    AMP,
    ICARUS,
    OZYS,
    PayloadActivation,
    Bulkhead,
}

#[derive(Parser, Debug)]
struct GenOtaKeyCli {
    secret_key_path: std::path::PathBuf,
    public_key_path: std::path::PathBuf,
}

#[tokio::main]
async fn main() -> Result<()> {
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Info)
        .try_init();
    let args = Cli::parse();
    println!("{:?}", args);

    match args.mode {
        ModeSelect::Download(args) => {
            if args.force_ota && args.force_probe {
                bail!("--force-ota and --force-probe can not be set at the same time")
            }

            let lister = Lister::new();
            let probes = lister.list_all();
            let use_probe = if args.force_probe {
                true
            } else if args.force_ota {
                false
            } else {
                probes.len() > 0
            };

            let (ready_tx, ready_rx) = oneshot::channel();
            let (logs_tx, logs_rx) = broadcast::channel(256);
            
            let download_future = async move {
                if use_probe {
                    info!("Using debug probe because there are 1 or more probes connected.");
                    download_probe(args, probes, ready_tx, logs_tx).await.unwrap();
                } else {
                    info!("Using OTA because there are no probe connected.");
                    todo!()
                }
            };

            let viewer_future = async move {
                ready_rx.await.unwrap();
                log_viewer_tui(logs_rx).await;
            };

            tokio::select! {
                _ = download_future => {},
                _ = viewer_future => {},
            }

            Ok(())
        }
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
    }
}
