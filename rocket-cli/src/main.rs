mod attach;
mod connect_method;
mod elf_locator;
mod gen_ota_key;
mod log_viewer;
mod probe;
mod bluetooth;

use anyhow::Result;
use attach::attach_target;
use bluetooth::ble_download::ble_download;
use bluetooth::extract_bin::extract_bin_and_sign;
use bluetooth::find_esp::ble_dispose;
use clap::Parser;
use clap::Subcommand;
use connect_method::ConnectMethod;
use gen_ota_key::gen_ota_key;
use log::LevelFilter;
use log_viewer::target_log::NodeTypeEnum;
use probe::probe_download::probe_download;

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

    #[command(about = "attach to target via probe or ota")]
    Attach(DownloadCli),

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
            let connect_method = ConnectMethod::new(&args).await?;
            match &connect_method {
                ConnectMethod::Probe(probe_string) => {
                    probe_download(&args, probe_string).await?;
                }
                ConnectMethod::OTA(esp) => {
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
            
            attach_target(&args, &connect_method).await?;

            if let ConnectMethod::OTA(esp) = connect_method {
                ble_dispose(esp).await?;
            }
            Ok(())
        }
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
    }
}
