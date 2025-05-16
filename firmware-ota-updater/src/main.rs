use std::process::Command;
use std::process::Output;

use anyhow::Ok;
use anyhow::Result;
use clap::Parser;
use clap::Subcommand;
use firmware_common_new::bootloader::generate_public_key;
use firmware_common_new::bootloader::sign_firmware;
use log::LevelFilter;
use log::debug;
use log::error;
use log::info;
use rand::Rng;
use salty::Sha512;
use tempfile::NamedTempFile;

#[derive(Parser)]
#[command(name = "Firmware OTA Updater")]
#[command(bin_name = "firmware-ota-updater")]
struct Cli {
    #[clap(subcommand)]
    mode: ModeSelect,
}

#[derive(Subcommand)]
enum ModeSelect {
    #[command(about = "sign and upload firmware")]
    Upload(UploadCli),

    #[command(about = "generate private and public keys for signing and verifying firmware")]
    GenKey(GenKeyCli),
}

#[derive(Parser)]
struct UploadCli {
    secret_path: std::path::PathBuf,
    firmware_elf_path: std::path::PathBuf,
}

#[derive(Parser)]
struct GenKeyCli {
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
        ModeSelect::Upload(args) => {
            let secret = std::fs::read(args.secret_path)?;
            if secret.len() != 32 {
                error!("Secret must be 32 bytes long.");
                return Ok(());
            }

            let objcopy_path = cargo_binutils::Tool::Objcopy
                .path()
                .expect("llvm-objcopy not found");
            debug!("objcopy path: {:?}", objcopy_path);

            let firmware_binary = NamedTempFile::new()?;
            let Output { status, stderr, .. } = Command::new(objcopy_path)
                .args(&[
                    args.firmware_elf_path.to_str().unwrap(),
                    "-O",
                    "binary",
                    firmware_binary.path().to_str().unwrap(),
                ])
                .output()?;
            if !status.success() {
                error!(
                    "objcopy failed (exit code {:?}): {}",
                    status.code(),
                    String::from_utf8_lossy(&stderr)
                );
                return Ok(());
            }

            let mut firmware_bytes = std::fs::read(firmware_binary.path())?;
            info!("Firmware size: {}KiB", firmware_bytes.len() / 1024);

            let mut sha512 = Sha512::new();
            sha512.update(&firmware_bytes);

            let signature: [u8; 64] =
                sign_firmware(&sha512.finalize(), secret.as_slice().try_into().unwrap());
            info!("Firmware signature: {}", hex::encode(signature));
            firmware_bytes.splice(0..0, signature);
        }
        ModeSelect::GenKey(args) => {
            let secret_key = rand::rng().random::<[u8; 32]>();
            let public_key = generate_public_key(&secret_key);
            std::fs::write(args.secret_key_path, &secret_key)?;
            std::fs::write(args.public_key_path, &public_key)?;
            info!("keys generated")
        }
    }

    Ok(())
}
