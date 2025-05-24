use crate::{
    DownloadCli,
    connect_method::ConnectMethod,
    log_viewer::{log_saver::LogSaver, log_viewer_tui, target_log::TargetLog},
    probe::{
        probe_attach::probe_attach,
    },
};
use anyhow::Result;
use tokio::sync::{broadcast, oneshot};

pub async fn attach_target(args: &DownloadCli, connect_method: &ConnectMethod) -> Result<()> {
    let mut log_saver = LogSaver::new(
        args.firmware_elf_path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string(),
    )
    .await?;

    let (logs_tx, logs_rx) = broadcast::channel::<TargetLog>(256);
    let mut logs_rx2 = logs_tx.subscribe();
    let (stop_tx, mut stop_rx) = oneshot::channel::<()>();

    let receive_log_future = async move {
        match connect_method {
            ConnectMethod::Probe(probe_string) => {
                probe_attach(args, probe_string, logs_tx, stop_rx).await.unwrap();
            }
            ConnectMethod::OTA => todo!(),
        }
    };

    let logger_future = async move {
        while let Ok(log) = logs_rx2.recv().await {
            log_saver.append_log(&log).await.unwrap();
        }
        log_saver.flush().await.unwrap();
    };

    let viewer_future = async move {
        log_viewer_tui(logs_rx).await.unwrap();
        stop_tx.send(()).unwrap();
    };

    tokio::join! {
        receive_log_future,
        logger_future,
        viewer_future,
    };

    Ok(())
}
