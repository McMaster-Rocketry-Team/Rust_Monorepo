use clap::Parser;
use clap::Subcommand;

use crate::log_viewer::target_log::NodeTypeEnum;

#[derive(Parser, Debug)]
#[command(name = "Rocket CLI")]
#[command(bin_name = "rocket-cli")]
pub struct Cli {
    #[clap(subcommand)]
    pub mode: ModeSelect,
}

#[derive(Subcommand, Debug)]
pub enum ModeSelect {
    #[command(about = "download firmware to target via probe or ota")]
    Download(DownloadCli),

    #[command(about = "attach to target via probe or ota")]
    Attach(DownloadCli),

    #[command(about = "generate private and public keys for ota")]
    GenOtaKey(GenOtaKeyCli),
}

#[derive(Parser, Debug)]
pub struct DownloadCli {
    #[arg(long, help = "force using ota")]
    pub force_ota: bool,
    #[arg(long, help = "force using probe")]
    pub force_probe: bool,
    pub chip: String,
    pub secret_path: std::path::PathBuf,
    pub node_type: NodeTypeEnum,
    pub firmware_elf_path: std::path::PathBuf,
}

#[derive(Parser, Debug)]
pub struct GenOtaKeyCli {
    pub secret_key_path: std::path::PathBuf,
    pub public_key_path: std::path::PathBuf,
}