use anyhow::Result;
use base64::prelude::*;
use clap::Parser;
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;

use crate::{
    bluetooth::{demultiplex_log::LogDemultiplexer, BluetoothConnectionMethod},
    elf_locator::locate_elf_files, monitor::target_log::TargetLog,
};

#[derive(Parser, Debug)]
pub struct DecodeBluetoothChunkArgs {
    pub base64: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ReturnValue {
    is_overrun: bool,
    logs: Vec<TargetLog>,
    messages: Vec<DecodedMessage>,
}

pub fn test_decode_bluetooth_chunk(args: DecodeBluetoothChunkArgs) -> Result<()> {
    let chunk = BASE64_STANDARD.decode(args.base64)?;

    let mut log_demultiplexer = LogDemultiplexer::new(locate_elf_files(None).unwrap_or_default());
    let (logs_tx, mut logs_rx) = broadcast::channel::<TargetLog>(256);
    let (messages_tx, mut messages_rx) = broadcast::channel::<DecodedMessage>(32);
    let is_overrun = BluetoothConnectionMethod::process_chunk(
        &chunk,
        &mut log_demultiplexer,
        &logs_tx,
        &messages_tx,
    )?;

    let mut logs = Vec::new();
    while let Ok(log) = logs_rx.try_recv() {
        logs.push(log);
    }

    let mut messages = Vec::new();
    while let Ok(message) = messages_rx.try_recv() {
        messages.push(message);
    }

    println!("{}", serde_json::to_string(&ReturnValue{
        is_overrun,
        logs,
        messages,
    })?);

    Ok(())
}
