use std::hint::black_box;

use crate::{
    args::DownloadCli, bluetooth::demultiplex_log::LogDemultiplexer, connect_method::ConnectMethod, elf_locator::locate_elf_files, log_viewer::{log_saver::LogSaver, log_viewer_tui, target_log::TargetLog, LogViewerStatus}, probe::probe_attach::probe_attach
};
use anyhow::Result;
use tokio::sync::{broadcast, oneshot, watch};

pub async fn attach_target(args: &DownloadCli, connect_method: &ConnectMethod) -> Result<()> {
    let mut log_saver = LogSaver::new(
        args.firmware_elf_path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string(),
        connect_method,
    )
    .await?;

    let (status_tx, status_rx) = watch::channel(LogViewerStatus::Initialize);
    let (logs_tx, logs_rx) = broadcast::channel::<TargetLog>(256);
    let mut logs_rx2 = logs_tx.subscribe();
    let (stop_tx, mut stop_rx) = oneshot::channel::<()>();

    let elf_info_map = if matches!(connect_method, ConnectMethod::OTA(_)) {
        Some(locate_elf_files()?)
    } else {
        None
    };

    let receive_log_future = async move {
        match connect_method {
            ConnectMethod::Probe(probe_string) => {
                probe_attach(args, probe_string, status_tx, logs_tx, stop_rx)
                    .await
                    .unwrap();
            }
            ConnectMethod::OTA(_) => {
                let elf_info_map = elf_info_map.unwrap();
                let mut log_demultiplexer = LogDemultiplexer::new(logs_tx, elf_info_map);

                // TODO
                while stop_rx.try_recv().is_err() {
                    let result = log_demultiplexer.process_chunk(black_box(&[]));
                    if result.is_error {
                        status_tx.send(LogViewerStatus::ChunkError).ok();
                    } else if result.is_overrun {
                        status_tx.send(LogViewerStatus::Overrun).ok();
                    } else {
                        status_tx.send(LogViewerStatus::Normal).ok();
                    }
                }
            }
        }
    };

    let logger_future = async move {
        while let Ok(log) = logs_rx2.recv().await {
            log_saver.append_log(&log).await.unwrap();
        }
        log_saver.flush().await.unwrap();
    };

    let viewer_future = async move {
        log_viewer_tui(logs_rx, status_rx).await.unwrap();
        stop_tx.send(()).unwrap();
    };

    tokio::join! {
        receive_log_future,
        logger_future,
        viewer_future,
    };

    Ok(())
}
