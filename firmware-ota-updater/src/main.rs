mod bluetooth;

use std::process::Command;
use std::process::Output;

use anyhow::Ok;
use anyhow::Result;
use bluetooth::ble_dispose;
use bluetooth::ble_get_peripheral;
use bluetooth::ble_send_file;
use clap::Parser;
use clap::Subcommand;
use firmware_common_new::bootloader::generate_public_key;
use firmware_common_new::bootloader::sign_firmware;
use firmware_common_new::can_bus::node_types::AMP_NODE_TYPE;
use firmware_common_new::can_bus::node_types::BULKHEAD_NODE_TYPE;
use firmware_common_new::can_bus::node_types::ICARUS_NODE_TYPE;
use firmware_common_new::can_bus::node_types::OZYS_NODE_TYPE;
use firmware_common_new::can_bus::node_types::PAYLOAD_ACTIVATION_NODE_TYPE;
use firmware_common_new::can_bus::node_types::VOID_LAKE_NODE_TYPE;
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
    node_type: NodeTypeEnum,
    firmware_elf_path: std::path::PathBuf,
}

#[derive(clap::ValueEnum, Clone)]
enum NodeTypeEnum {
    VoidLake,
    AMP,
    ICARUS,
    OZYS,
    PayloadActivation,
    Bulkhead,
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
            info!("Firmware size: {}bytes", firmware_bytes.len());
            if firmware_bytes.len() % 8 != 0 {
                error!("Firmware size is not a multiple of 8!");
                return Ok(());
            }

            let mut sha512 = Sha512::new();
            sha512.update(&firmware_bytes);

            let signature: [u8; 64] =
                sign_firmware(&sha512.finalize(), secret.as_slice().try_into().unwrap());
            info!("Firmware signature: {}", hex::encode(signature));
            firmware_bytes.splice(0..0, signature);

            let node_type = match args.node_type {
                NodeTypeEnum::VoidLake => VOID_LAKE_NODE_TYPE,
                NodeTypeEnum::AMP => AMP_NODE_TYPE,
                NodeTypeEnum::ICARUS => ICARUS_NODE_TYPE,
                NodeTypeEnum::OZYS => OZYS_NODE_TYPE,
                NodeTypeEnum::PayloadActivation => PAYLOAD_ACTIVATION_NODE_TYPE,
                NodeTypeEnum::Bulkhead => BULKHEAD_NODE_TYPE,
            };

            let esp = ble_get_peripheral().await?;
            ble_send_file(&esp, &firmware_bytes, Some(node_type), None).await?;
            ble_dispose(esp).await?;
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
