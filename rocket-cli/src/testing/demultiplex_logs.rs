use anyhow::Result;
use base64::prelude::*;
use clap::Parser;
use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;

use crate::{
    bluetooth::demultiplex_log::{LogDemultiplexer, ProcessChunkResult},
    elf_locator::locate_elf_files,
    log_viewer::target_log::TargetLog,
};

#[derive(Parser, Debug)]
pub struct DemultiPlexLogsArgs {
    pub base64: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ReturnValue {
    process_result: ProcessChunkResult,
    logs: Vec<TargetLog>,
}

pub fn demultiplex_logs(args: DemultiPlexLogsArgs) -> Result<()> {
    let chunk = BASE64_STANDARD.decode(args.base64)?;

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
