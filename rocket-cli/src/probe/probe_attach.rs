use crate::DownloadCli;
use crate::log_viewer::target_log::{DefmtLogInfo, TargetLog, parse_log_level};
use anyhow::Result;
use regex::Regex;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tokio::sync::{broadcast, oneshot};

pub async fn probe_attach(
    args: &DownloadCli,
    probe_string: &String,
    logs_tx: broadcast::Sender<TargetLog>,
    stop_rx: oneshot::Receiver<()>,
) -> Result<()> {
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
        ">>>>>{s}|||||{F}|||||{l}|||||{L}|||||{m}|||||{t}<<<<<",
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
    let re = Regex::new(r">>>>>(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)\|\|\|\|\|(.*?)<<<<<").unwrap();

    let read_logs_future = async move {
        while let Some(line) = lines.next_line().await.unwrap() {
            if let Some(cap) = re.captures(&line) {
                let log = TargetLog {
                    node_type: args.node_type,
                    node_id: None,
                    log_content: cap.get(1).unwrap().as_str().to_string(),
                    defmt: Some(DefmtLogInfo {
                        file_path: cap.get(2).unwrap().as_str().to_string(),
                        line_number: cap.get(3).unwrap().as_str().to_string(),
                        log_level: parse_log_level(cap.get(4).unwrap().as_str()),
                        module_path: cap.get(5).unwrap().as_str().to_string(),
                        timestamp: cap.get(6).unwrap().as_str().parse::<f64>().ok(),
                    }),
                };
                if logs_tx.send(log).is_err() {
                    break;
                }
            }
        }
    };

    tokio::select! {
        _ = read_logs_future => {},
        _ = stop_rx => {},
    }

    child.kill().await?;

    Ok(())
}
