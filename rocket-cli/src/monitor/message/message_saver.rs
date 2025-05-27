use anyhow::Result;
use chrono::{DateTime, Local};
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use sanitise_file_name::sanitise;
use std::{path::PathBuf, time::Instant};
use tokio::{
    fs::{self, File},
    io::AsyncWriteExt,
};

use crate::connection_method::ConnectionMethod;

pub struct CanMessageSaver {
    file: File,
    start_time: Instant,
}

impl CanMessageSaver {
    pub async fn new(
        start_time: (DateTime<Local>, Instant),
        bin_name: &str,
        connection_method: &mut Box<dyn ConnectionMethod>,
    ) -> Result<Self> {
        let logs_dir = PathBuf::from("logs");
        fs::create_dir_all(&logs_dir).await?;

        let timestamp = start_time.0.format("%Y-%m-%d_%H-%M-%S");
        let log_path = logs_dir.join(format!(
            "{}_{}_{}.message.log",
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

    pub async fn append_message(&mut self, message: &DecodedMessage) -> Result<()> {
        let elapsed = self.start_time.elapsed();
        let seconds = elapsed.as_secs();
        let nanos = elapsed.subsec_nanos();
        let relative_time = format!("{:05}.{:06}", seconds, nanos / 1000);

        self.file
            .write_all(
                &format!(
                    "{} {}\n",
                    relative_time,
                    serde_json::to_string(&message).unwrap(),
                )
                .as_bytes(),
            )
            .await?;
        Ok(())
    }

    pub async fn flush(&mut self) -> Result<()> {
        self.file.flush().await?;
        Ok(())
    }
}
