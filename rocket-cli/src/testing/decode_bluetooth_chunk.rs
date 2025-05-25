use anyhow::{Result, anyhow, bail};
use base64::prelude::*;
use clap::Parser;
use firmware_common_new::can_bus::telemetry::message_aggregator::{
    DecodedMessage, decode_aggregated_can_bus_messages,
};
use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;

use crate::{
    bluetooth::demultiplex_log::LogDemultiplexer, elf_locator::locate_elf_files,
    log_viewer::target_log::TargetLog,
};

#[derive(Parser, Debug)]
pub struct DecodeBluetoothChunkArgs {
    pub base64: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
enum ReturnValue {
    AggregatedMessages {
        is_overrun: bool,
        messages: Vec<DecodedMessage>,
    },
    DemultiplexedLogs {
        is_overrun: bool,
        logs: Vec<TargetLog>,
    },
}

pub fn test_decode_bluetooth_chunk(args: DecodeBluetoothChunkArgs) -> Result<()> {
    let chunk = BASE64_STANDARD.decode(args.base64)?;

    let chunk_type = chunk[0] << 6;
    let return_value = match chunk_type {
        0b00 => test_demultiplex_logs(&chunk)?,
        0b01 => test_decode_aggregated_messages(&chunk)?,
        _ => bail!("Invalid message"),
    };

    println!("{}", serde_json::to_string(&return_value)?);

    Ok(())
}

fn test_demultiplex_logs(chunk: &[u8]) -> Result<ReturnValue> {
    let (logs_tx, mut logs_rx) = broadcast::channel::<TargetLog>(256);
    let mut log_demultiplexer =
        LogDemultiplexer::new(logs_tx, locate_elf_files().unwrap_or_default());
    let is_overrun = log_demultiplexer.process_chunk(&chunk)?;

    let mut logs = Vec::new();
    while let Ok(log) = logs_rx.try_recv() {
        logs.push(log);
    }

    Ok(ReturnValue::DemultiplexedLogs { is_overrun, logs })
}

fn test_decode_aggregated_messages(chunk: &[u8]) -> Result<ReturnValue> {
    let mut messages = Vec::<DecodedMessage>::new();
    let is_overrun = decode_aggregated_can_bus_messages(&chunk, |message| messages.push(message))
        .map_err(|_| anyhow!("Invalid message"))?;

    Ok(ReturnValue::AggregatedMessages {
        is_overrun,
        messages,
    })
}
