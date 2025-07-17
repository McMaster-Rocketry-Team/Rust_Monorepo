use std::{path::PathBuf, process::Output};
use base64::prelude::*;

use anyhow::{Ok, Result, anyhow, bail};
use firmware_common_new::bootloader::sign_firmware;
use log::info;
use salty::Sha512;
use tempfile::NamedTempFile;

pub async fn extract_bin_and_sign(
    secret_path: &PathBuf,
    firmware_elf_path: &PathBuf,
) -> Result<Vec<u8>> {
    let secret = std::fs::read(secret_path)?;
    let secret = BASE64_STANDARD.decode(&secret)?;
    if secret.len() != 32 {
        bail!("Secret must be 32 bytes long.");
    }

    let objcopy_path = cargo_binutils::Tool::Objcopy.path().map_err(|_| {
        anyhow!(
            "llvm-objcopy not found, Please install it by running 'rustup component add llvm-tools'"
        )
    })?;

    let firmware_binary = NamedTempFile::new()?;
    let Output { status, stderr, .. } = std::process::Command::new(objcopy_path)
        .args(&[
            firmware_elf_path.to_str().unwrap(),
            "-O",
            "binary",
            firmware_binary.path().to_str().unwrap(),
        ])
        .output()?;
    if !status.success() {
        bail!(
            "objcopy failed (exit code {:?}): {}",
            status.code(),
            String::from_utf8_lossy(&stderr)
        );
    }

    let mut firmware_bytes = std::fs::read(firmware_binary.path())?;
    info!("Firmware size: {}bytes", firmware_bytes.len());
    if firmware_bytes.len() % 8 != 0 {
        bail!("Firmware size is not a multiple of 8!");
    }

    let mut sha512 = Sha512::new();
    sha512.update(&firmware_bytes);

    let signature: [u8; 64] =
        sign_firmware(&sha512.finalize(), secret.as_slice().try_into().unwrap());
    firmware_bytes.splice(0..0, signature);
    info!("Firmware signed successfully");

    Ok(firmware_bytes)
}
