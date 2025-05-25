use std::{collections::HashMap, env, mem::transmute, path::PathBuf};

use anyhow::{Result, anyhow};
use defmt_decoder::{DecodeError, StreamDecoder};
use firmware_common_new::can_bus::telemetry::log_multiplexer::decode_multiplexed_log_chunk;
use log::Level;
use tokio::sync::broadcast;

use crate::{
    args::NodeTypeEnum,
    elf_locator::{DefmtElfInfo, ELFInfoMap},
    monitor::target_log::{DefmtLocationInfo, DefmtLogInfo, TargetLog},
};

pub struct LogDemultiplexer {
    current_dir: Option<PathBuf>,
    elf_info_map: ELFInfoMap,
    /// node id -> line buffer
    plain_text_log_line_buffers: HashMap<u16, LineBuffer>,

    /// node id -> defmt decoder
    defmt_decoders: HashMap<u16, Box<dyn StreamDecoder>>,
}

impl LogDemultiplexer {
    pub fn new(elf_info_map: ELFInfoMap) -> Self {
        Self {
            current_dir: env::current_dir().ok(),
            elf_info_map,
            plain_text_log_line_buffers: HashMap::new(),
            defmt_decoders: HashMap::new(),
        }
    }

    /// chunk from LogMultiplexer::create_chunk
    /// returns is_overrun
    pub fn process_chunk(
        &mut self,
        chunk: &[u8],
        logs_tx: &broadcast::Sender<TargetLog>,
    ) -> Result<bool> {
        decode_multiplexed_log_chunk(chunk, |frame| {
            let node_type: NodeTypeEnum = frame.node_type.into();
            if let Some(elf_info) = self.elf_info_map.get(&node_type) {
                // treat bytes as defmt log

                let defmt_decoder = self.defmt_decoders.entry(frame.node_id).or_insert_with(|| {
                    // SAFETY: we know the reference to elf_info stored in the pinned box will live as long as LogDemultiplexer is not dropped
                    let elf_info: &DefmtElfInfo = elf_info.as_ref().get_ref();
                    let elf_info: &'static DefmtElfInfo = unsafe { transmute(elf_info) };
                    elf_info.table.new_stream_decoder()
                });

                defmt_decoder.received(&frame.data);

                loop {
                    match defmt_decoder.decode() {
                        Ok(defmt_frame) => {
                            let mut location_info: Option<DefmtLocationInfo> = None;
                            let loc = elf_info
                                .locs
                                .as_ref()
                                .map(|locs| locs.get(&defmt_frame.index()));

                            if let Some(Some(loc)) = loc {
                                // try to get the relative path, else the full one
                                let path = if let Some(current_dir) = &self.current_dir {
                                    loc.file.strip_prefix(current_dir).unwrap_or(&loc.file)
                                } else {
                                    &loc.file
                                };

                                location_info = Some(DefmtLocationInfo {
                                    module_path: loc.module.clone(),
                                    file_path: path.display().to_string(),
                                    line_number: loc.line.to_string(),
                                });
                            }

                            let timestamp = defmt_frame
                                .display_timestamp()
                                .map(|ts| ts.to_string().parse::<f64>().ok())
                                .flatten();
                            let log_level = defmt_frame
                                .level()
                                .map(|level| match level {
                                    defmt_parser::Level::Trace => Level::Trace,
                                    defmt_parser::Level::Debug => Level::Debug,
                                    defmt_parser::Level::Info => Level::Info,
                                    defmt_parser::Level::Warn => Level::Warn,
                                    defmt_parser::Level::Error => Level::Error,
                                })
                                .unwrap_or(Level::Info);
                            let log_content = defmt_frame.display_message().to_string();
                            logs_tx
                                .send(TargetLog {
                                    node_type,
                                    node_id: Some(frame.node_id),
                                    log_content,
                                    defmt: Some(DefmtLogInfo {
                                        log_level,
                                        timestamp,
                                        location: location_info,
                                    }),
                                })
                                .ok();
                        }
                        Err(DecodeError::UnexpectedEof) => break,
                        Err(DecodeError::Malformed) => {
                            if elf_info.table.encoding().can_recover() {
                                continue;
                            } else {
                                break;
                            }
                        }
                    }
                }
            } else {
                // treat bytes as plain text

                let line_buffer = self
                    .plain_text_log_line_buffers
                    .entry(frame.node_id)
                    .or_default();
                line_buffer.push_bytes(&frame.data, |line| {
                    logs_tx
                        .send(TargetLog {
                            node_type,
                            node_id: Some(frame.node_id),
                            log_content: line,
                            defmt: None,
                        })
                        .ok();
                });
            }
        })
        .map_err(|e| anyhow!("{:?}", e))
    }
}

#[derive(Default, Debug, Clone)]
struct LineBuffer {
    buffer: Vec<u8>,
}

impl LineBuffer {
    fn push_bytes(&mut self, data: &[u8], mut on_line: impl FnMut(String)) {
        for c in data.iter() {
            if *c as char == '\r' || *c as char == '\n' {
                if !self.buffer.is_empty() {
                    on_line(String::from_utf8_lossy(&self.buffer).into())
                }
            } else {
                self.buffer.push(*c);
            }
        }
    }
}
