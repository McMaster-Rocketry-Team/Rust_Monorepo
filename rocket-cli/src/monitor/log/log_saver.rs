use anyhow::Result;
use chrono::{DateTime, Local};
use pad::PadStr as _;
use sanitise_file_name::sanitise;
use std::path::PathBuf;
use std::time::Instant;
use tokio::{
    fs::{self, File},
    io::AsyncWriteExt,
};

use crate::connection_method::ConnectionMethod;

use super::target_log::TargetLog;

pub struct LogSaver {
    file: File,
    start_time: Instant,
}

impl LogSaver {
    pub async fn new(
        start_time: (DateTime<Local>, Instant),
        bin_name: &str,
        connection_method: &mut Box<dyn ConnectionMethod>,
    ) -> Result<Self> {
        let logs_dir = PathBuf::from("logs");
        fs::create_dir_all(&logs_dir).await?;

        let timestamp = start_time.0.format("%Y-%m-%d_%H-%M-%S");
        let log_path = logs_dir.join(format!(
            "{}_{}_{}.log",
            timestamp,
            bin_name,
            sanitise(
                &connection_method
                    .name()
                    .replace(":", "_")
                    .replace(" ", "_")
                    .to_lowercase()
            ),
        ));
        let file = File::create(log_path).await?;

        Ok(Self {
            file,
            start_time: start_time.1,
        })
    }

    pub async fn append_log(&mut self, log: &TargetLog) -> Result<()> {
        let elapsed = self.start_time.elapsed();
        let seconds = elapsed.as_secs();
        let nanos = elapsed.subsec_nanos();
        let relative_time = format!("{:05}.{:06}", seconds, nanos / 1000);

        self.file
            .write_all(
                &format!(
                    "{} {:<3} {} ",
                    relative_time,
                    &log.node_type.short_name(),
                    &log.node_id.map_or(String::from("xxx"), |id| format!("{:0>3X}", id)),
                )
                .as_bytes(),
            )
            .await?;

        if let Some(defmt_info) = &log.defmt {
            self.file
                .write_all(
                    &format!("[{}]", &defmt_info.log_level.to_string())
                        .pad_to_width(8)
                        .as_bytes(),
                )
                .await?;

            if let Some(timestamp) = &defmt_info.timestamp {
                self.file
                    .write_all(&format!("{:.6} ", &timestamp).as_bytes())
                    .await?;
            }
        }

        self.file
            .write_all(&format!("{}\n", &log.log_content).as_bytes())
            .await?;

        Ok(())
    }

    pub async fn flush(&mut self) -> Result<()> {
        self.file.flush().await?;
        Ok(())
    }
}
