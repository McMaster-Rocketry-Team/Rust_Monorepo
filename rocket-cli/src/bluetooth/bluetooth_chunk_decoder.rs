use anyhow::{Result, bail};
use tokio::sync::broadcast;

use crate::{elf_locator::ELFInfoMap, log_viewer::target_log::TargetLog};

use super::demultiplex_log::LogDemultiplexer;

pub struct BluetoothChunkDecoder {
    log_demultiplexer: LogDemultiplexer,
}

impl BluetoothChunkDecoder {
    pub fn new(logs_tx: broadcast::Sender<TargetLog>, elf_info_map: ELFInfoMap) -> Self {
        Self {
            log_demultiplexer: LogDemultiplexer::new(logs_tx, elf_info_map),
        }
    }

    // returns is_overrun
    pub fn process_chunk(&mut self, chunk: &[u8]) -> Result<bool> {
        if chunk.len() == 0 {
            bail!("Invalid bluetooth chunk");
        }

        let chunk_type = chunk[0] << 6;
        match chunk_type {
            0b00 => self.log_demultiplexer.process_chunk(chunk),
            0b01 => {
                todo!()
            }
            _ => bail!("Invalid bluetooth chunk"),
        }
    }
}
