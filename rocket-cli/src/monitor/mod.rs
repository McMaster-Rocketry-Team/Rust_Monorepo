mod log;
mod message;
mod config;

pub use log::LogViewerStatus;
pub use log::target_log;

use crate::{args::NodeTypeEnum, connect_method::ConnectionMethod};
use anyhow::Result;
use std::path::PathBuf;

pub async fn monitor_tui(
    connection_method: &mut Box<dyn ConnectionMethod>,
    chip: &String,
    secret_path: &PathBuf,
    node_type: &NodeTypeEnum,
    firmware_elf_path: &PathBuf,
) -> Result<()> {
    Ok(())
}
