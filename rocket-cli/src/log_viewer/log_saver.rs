use crate::{connect_method::ConnectMethod, log_viewer::target_log::TargetLog};
use anyhow::{Ok, Result};
use chrono::Local;
use pad::PadStr as _;
use std::path::PathBuf;
use tokio::{
    fs::{self, File},
    io::AsyncWriteExt,
};

pub struct LogSaver {
    file: File,
}

impl LogSaver {
    pub async fn new(bin_name: String, connect_method: &ConnectMethod) -> Result<Self> {
        let logs_dir = PathBuf::from("logs");
        fs::create_dir_all(&logs_dir).await?;

        let timestamp = Local::now().format("%Y-%m-%d_%H-%M-%S");
        let log_path = logs_dir.join(format!(
            "{}_{}_{}.log",
            timestamp,
            bin_name,
            match connect_method {
                ConnectMethod::Probe(_) => "probe",
                ConnectMethod::OTA(_) => "ota",
            }
        ));
        let file = File::create(log_path).await?;

        Ok(Self { file })
    }

    pub async fn append_log(&mut self, log: &TargetLog) -> Result<()> {
        self.file
            .write_all(
                &format!(
                    "{} {} ",
                    &log.node_type.short_name().pad_to_width(4),
                    &log.node_id.map_or(String::from("xxx"), |id| {
                        format!("{:X}", id).pad(3, '0', pad::Alignment::Right, false)
                    }),
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
                    .write_all(&format!("{:.6} ", &timestamp,).as_bytes())
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
