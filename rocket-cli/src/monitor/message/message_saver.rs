use anyhow::Result;
use chrono::{DateTime, Local};
use firmware_common_new::can_bus::{messages::CanBusMessageEnum, telemetry::message_aggregator::DecodedMessage};
use sanitise_file_name::sanitise;
use std::{path::PathBuf, time::Instant};
use tokio::{
    fs::{self, File},
    io::AsyncWriteExt,
};

use crate::connection_method::ConnectionMethod;

pub struct CanMessageSaver {
    file: File,
    imu_csv_file: File,
    start_time: Instant,
    imu_csv_header_written: bool,
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

        // Create IMU CSV file
        let imu_csv_path = logs_dir.join(format!(
            "{}_{}_{}.imu.csv",
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
        let imu_csv_file = File::create(imu_csv_path).await?;

        Ok(Self {
            file,
            imu_csv_file,
            start_time: start_time.1,
            imu_csv_header_written: false,
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

        // Handle IMU measurements and save to CSV
        if let CanBusMessageEnum::IMUMeasurement(imu_msg) = &message.message {
            // Write CSV header if not already written
            if !self.imu_csv_header_written {
                self.imu_csv_file
                    .write_all(
                        "relative_time_s,timestamp_us,acc_x_mps2,acc_y_mps2,acc_z_mps2,gyro_x_degs,gyro_y_degs,gyro_z_degs\n"
                            .as_bytes(),
                    )
                    .await?;
                self.imu_csv_header_written = true;
            }

            // Extract IMU data
            let acc = imu_msg.acc();
            let gyro = imu_msg.gyro();
            let timestamp_us = imu_msg.timestamp_us;

            // Write CSV row
            let csv_line = format!(
                "{},{},{},{},{},{},{},{}\n",
                relative_time,
                timestamp_us,
                acc.x,
                acc.y,
                acc.z,
                gyro.x,
                gyro.y,
                gyro.z
            );
            self.imu_csv_file.write_all(csv_line.as_bytes()).await?;
        }
        Ok(())
    }

    pub async fn flush(&mut self) -> Result<()> {
        self.file.flush().await?;
        self.imu_csv_file.flush().await?;
        Ok(())
    }
}
