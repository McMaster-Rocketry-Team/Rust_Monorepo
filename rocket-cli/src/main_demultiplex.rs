#![allow(dead_code)]

mod args;
mod attach;
mod bluetooth;
mod connect_method;
mod elf_locator;
mod gen_ota_key;
mod log_viewer;
mod probe;

use anyhow::{Result, anyhow};
use base64::prelude::*;
use bluetooth::demultiplex_log::{LogDemultiplexer, ProcessChunkResult};
use elf_locator::locate_elf_files;
use log_viewer::target_log::TargetLog;
use serde::{Deserialize, Serialize};
use std::env;
use tokio::sync::broadcast;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ReturnValue {
    process_result: ProcessChunkResult,
    logs: Vec<TargetLog>,
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    let arg1 = args.get(1).ok_or(anyhow!("expect 1 arg"))?;
    let chunk = BASE64_STANDARD.decode(arg1)?;

    let (logs_tx, mut logs_rx) = broadcast::channel::<TargetLog>(256);
    let mut log_demultiplexer =
        LogDemultiplexer::new(logs_tx, locate_elf_files().unwrap_or_default());
    let process_result = log_demultiplexer.process_chunk(&chunk);

    let mut return_value = ReturnValue {
        process_result,
        logs: Vec::new(),
    };

    while let Ok(log) = logs_rx.try_recv() {
        return_value.logs.push(log);
    }

    println!("{}", serde_json::to_string(&return_value)?);

    Ok(())
}
