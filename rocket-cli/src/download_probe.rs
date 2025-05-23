use std::process;

use crate::DownloadCli;
use crate::target_log::TargetLog;
use anyhow::{Result, bail};
use log::info;
use probe_rs::probe::DebugProbeInfo;
use prompted::input;
use regex::Regex;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tokio::signal;
use tokio::sync::{broadcast, oneshot};

pub async fn download_probe(
    args: DownloadCli,
    probes: Vec<DebugProbeInfo>,
    ready_tx: oneshot::Sender<()>,
    logs_tx: broadcast::Sender<TargetLog>,
) -> Result<()> {
    let probe = if probes.len() == 1 {
        probes[0].clone()
    } else {
        for i in 0..probes.len() {
            let probe = &probes[i];

            println!(
                "[{}]: {}, SN {}",
                i + 1,
                probe.identifier,
                probe.serial_number.clone().unwrap_or("N/A".into())
            );
        }

        let selection = input!("Select one probe (1-{}): ", probes.len());

        let selection: usize = match selection.trim().parse() {
            Err(_) => bail!("Invalid selection"),
            Ok(num) if num > probes.len() => bail!("Invalid selection"),
            Ok(num) => num,
        };

        probes[selection].clone()
    };

    let probe_string = format!(
        "{:x}:{:x}{}",
        probe.vendor_id,
        probe.product_id,
        probe
            .serial_number
            .map_or(String::new(), |sn| format!(":{}", sn))
    );

    // flash the firmware
    // let probe_rs_args = [
    //     "download",
    //     "--non-interactive",
    //     "--probe",
    //     &probe_string,
    //     "--chip",
    //     &args.chip,
    //     "--connect-under-reset",
    //     args.firmware_elf_path.to_str().unwrap(),
    // ];
    // let output = std::process::Command::new("probe-rs")
    //     .args(&probe_rs_args)
    //     .status()?;

    // if !output.success() {
    //     bail!("probe-rs command failed");
    // }
    ready_tx.send(()).unwrap();

    // attach to the target
    let probe_rs_args = [
        "attach",
        "--non-interactive",
        "--probe",
        &probe_string,
        "--chip",
        &args.chip,
        "--connect-under-reset",
        "--log-format",
        ">>>>>{s}|||||{c}|||||{f}|||||{F}|||||{l}|||||{L}|||||{m}|||||{t}<<<<<",
        args.firmware_elf_path.to_str().unwrap(),
    ];

    let mut child = Command::new("probe-rs")
        .args(&probe_rs_args)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::inherit())
        .spawn()?;

    let stdout = child.stdout.take().unwrap();
    let reader = BufReader::new(stdout);
    let mut lines = reader.lines();
    let re = Regex::new(r">>>>>(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)<<<<<").unwrap();

    while let Some(line) = lines.next_line().await? {
        if let Some(cap) = re.captures(&line) {
            let log = TargetLog {
                log_content: cap.get(1).unwrap().as_str().to_string(),
                crate_name: cap.get(2).unwrap().as_str().to_string(),
                file_name: cap.get(3).unwrap().as_str().to_string(),
                file_path: cap.get(4).unwrap().as_str().to_string(),
                line_number: cap.get(5).unwrap().as_str().to_string(),
                log_level: cap.get(6).unwrap().as_str().to_string(),
                module_path: cap.get(7).unwrap().as_str().to_string(),
                timestamp: cap.get(8).unwrap().as_str().to_string(),
            };
            if logs_tx.send(log).is_err() {
                break;
            }
        }
    }

    Ok(())
}
