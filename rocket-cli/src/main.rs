mod config;
mod download_probe;
mod gen_ota_key;
mod log_viewer_tui;
mod target_log;

use std::time::Duration;

use anyhow::Result;
use anyhow::bail;
use clap::Parser;
use clap::Subcommand;
use download_probe::download_probe;
use gen_ota_key::gen_ota_key;
use log::Level;
use log::LevelFilter;
use log::info;
use log_viewer_tui::log_viewer_tui;
use probe_rs::probe::list::Lister;
use target_log::DefmtLogInfo;
use target_log::NodeTypeEnum;
use target_log::TargetLog;
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

            let (ready_tx, ready_rx) = oneshot::channel::<()>();
            let (logs_tx, logs_rx) = broadcast::channel::<TargetLog>(256);

            let download_future = async move {
                if cfg!(debug_assertions) {
                    ready_tx.send(()).unwrap();

                    loop {
                        logs_tx
                            .send(TargetLog {
                                node_type: NodeTypeEnum::VoidLake,
                                node_id: Some(0xFFF),
                                log_content: "Hello VLF5!".to_string(),
                                defmt: Some(DefmtLogInfo {
                                    file_path: "".to_string(),
                                    line_number: "".to_string(),
                                    log_level: Level::Trace,
                                    module_path: "".to_string(),
                                    timestamp: Some(1.0),
                                }),
                            })
                            .unwrap();
                        logs_tx
                            .send(TargetLog {
                                node_type: NodeTypeEnum::VoidLake,
                                node_id: Some(20),
                                log_content: "Hello VLF5!".to_string(),
                                defmt: None,
                            })
                            .unwrap();
                        logs_tx
                            .send(TargetLog {
                                node_type: NodeTypeEnum::VoidLake,
                                node_id: None,
                                log_content:
                                    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                                        .to_string(),
                                defmt: None,
                            })
                            .unwrap();
                        logs_tx
                            .send(TargetLog {
                                node_type: NodeTypeEnum::ICARUS,
                                node_id: None,
                                log_content: "Hello ICARUS!".to_string(),
                                defmt: None,
                            })
                            .unwrap();
                        logs_tx
                            .send(TargetLog {
                                node_type: NodeTypeEnum::ICARUS,
                                node_id: None,
                                log_content: "Hello ICARUS!".to_string(),
                                defmt: None,
                            })
                            .unwrap();
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                } else {
                    if use_probe {
                        info!("Using debug probe because there are 1 or more probes connected.");
                        download_probe(args, probes, ready_tx, logs_tx)
                            .await
                            .unwrap();
                    } else {
                        info!("Using OTA because there are no probe connected.");
                        todo!()
                    }
                }
            };

            let viewer_future = async move {
                ready_rx.await.unwrap();
                log_viewer_tui(logs_rx).await.unwrap();
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
