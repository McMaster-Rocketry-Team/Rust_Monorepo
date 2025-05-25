use crate::DownloadCli;
use anyhow::{Result, bail};

pub fn check_probe_rs_installed() -> Result<()> {
    let output = std::process::Command::new("probe-rs")
        .arg("--version")
        .output();

    if output.is_err() {
        bail!("probe-rs not found. Please install it by running 'cargo install probe-rs-tools --locked'");
    }

    Ok(())
}

pub async fn probe_download(args: &DownloadCli, probe_string: &String) -> Result<()> {
    // flash the firmware
    let probe_rs_args = [
        "download",
        "--non-interactive",
        "--probe",
        &probe_string,
        "--chip",
        &args.chip,
        "--connect-under-reset",
        args.firmware_elf_path.to_str().unwrap(),
    ];
    let output = std::process::Command::new("probe-rs")
        .args(&probe_rs_args)
        .status()?;

    if !output.success() {
        bail!("probe-rs command failed");
    }

    Ok(())
}
