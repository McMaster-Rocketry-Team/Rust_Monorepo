mod config;
mod log;
mod message;
mod node;
mod status_bar;

use chrono::Local;
use config::MonitorConfig;
use cursive::{
    theme::{Palette, PaletteStyle},
    view::{Nameable, Resizable},
    views::{BoxedView, Dialog, HideableView, LinearLayout, TextView},
};
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
pub use log::target_log;
use log::{LogViewer, log_saver::LogSaver};
pub use message::message_row::FieldWidget;
use message::{CanMessageViewer, message_saver::CanMessageSaver};
use node::NodeStatusViewer;
use status_bar::{SelectedTab, StatusBar};
use tokio::{
    sync::{broadcast, oneshot, watch},
    task::spawn_blocking,
};

use crate::{connection_method::ConnectionMethod, enable_stdout_logging};
use anyhow::Result;
use std::{
    path::PathBuf,
    sync::{Arc, RwLock},
    time::Instant,
};

use self::target_log::TargetLog;

#[derive(Debug, Clone, Copy)]
pub enum MonitorStatus {
    Initialize,
    Normal,
}

pub async fn monitor_tui(
    connection_method: &mut Box<dyn ConnectionMethod>,
    firmware_elf_path: Option<&PathBuf>,
) -> Result<()> {
    let start_time = (Local::now(), Instant::now());
    let bin_name = if let Some(firmware_elf_path) = firmware_elf_path {
        firmware_elf_path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string()
    } else {
        "unknown".to_string()
    };
    let log_saver = LogSaver::new(start_time, &bin_name, connection_method).await?;
    let message_saver = CanMessageSaver::new(start_time, &bin_name, connection_method).await?;

    let (status_tx, mut status_rx) = watch::channel(MonitorStatus::Initialize);
    let (logs_tx, logs_rx) = broadcast::channel::<TargetLog>(256);
    let logs_rx2 = logs_tx.subscribe();
    let (messages_tx, messages_rx) = broadcast::channel::<DecodedMessage>(32);
    let messages_rx2 = messages_tx.subscribe();
    let (stop_tx, stop_rx) = oneshot::channel::<()>();

    let name = connection_method.name();
    let tui_future = async {
        status_rx.changed().await.unwrap();

        spawn_blocking(move || tui_task(name, status_rx, logs_rx, messages_rx, stop_tx)).await
    };

    let attach_future = connection_method.attach(status_tx, logs_tx, messages_tx, stop_rx);

    let log_saver_future = log_saver_task(log_saver, logs_rx2);
    let message_saver_future = message_saver_task(message_saver, messages_rx2);

    let (attach_result, tui_result, log_saver_result, message_saver_result) = tokio::join!(
        attach_future,
        tui_future,
        log_saver_future,
        message_saver_future
    );
    attach_result?;
    tui_result??;
    log_saver_result?;
    message_saver_result?;

    Ok(())
}

fn tui_task(
    connection_method_name: String,
    status_rx: watch::Receiver<MonitorStatus>,
    logs_rx: broadcast::Receiver<TargetLog>,
    messages_rx: broadcast::Receiver<DecodedMessage>,
    stop_tx: oneshot::Sender<()>,
) -> Result<()> {
    let first_time = !MonitorConfig::exists();
    let config = Arc::new(RwLock::new(MonitorConfig::load()?));
    let messages_rx2 = messages_rx.resubscribe();

    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();
    theme.palette = Palette::terminal_default();
    theme.palette[PaletteStyle::EditableTextCursor] = theme.palette[PaletteStyle::EditableText];
    theme.palette[PaletteStyle::EditableText] = theme.palette[PaletteStyle::Primary];
    siv.set_theme(theme);
    siv.set_autorefresh(true);

    siv.add_fullscreen_layer(
        LinearLayout::vertical()
            .child(
                StatusBar::new(connection_method_name, status_rx)
                    .with_name("status_bar")
                    .fixed_height(1)
                    .full_width(),
            )
            .child(
                HideableView::new(BoxedView::boxed(
                    LogViewer::new(config, logs_rx)
                        .with_name("log_viewer")
                        .full_screen(),
                ))
                .visible(true)
                .with_name("log_viewer_hideable"),
            )
            .child(
                HideableView::new(BoxedView::boxed(
                    CanMessageViewer::new(messages_rx)
                        .with_name("can_message_viewer")
                        .full_screen(),
                ))
                .visible(false)
                .with_name("message_viewer_hideable"),
            )
            .child(
                HideableView::new(BoxedView::boxed(
                    NodeStatusViewer::new(messages_rx2)
                        .with_name("node_status_viewer")
                        .full_screen(),
                ))
                .visible(false)
                .with_name("node_status_hideable"),
            ),
    );

    if first_time {
        siv.add_layer(
            Dialog::around(TextView::new("Click on the log to view the line number"))
                .title("Tips")
                .button("OK", |s| {
                    s.pop_layer().unwrap();
                }),
        );
    }

    enable_stdout_logging(false);
    let mut runner = siv.runner();
    runner.refresh();

    while runner.is_running() {
        {
            let mut log_viewer = runner.find_name::<LogViewer>("log_viewer").unwrap();
            log_viewer.receive_logs();

            let mut can_message_viewer = runner
                .find_name::<CanMessageViewer>("can_message_viewer")
                .unwrap();
            can_message_viewer.receive_messages();

            let mut node_status_viewer = runner
                .find_name::<NodeStatusViewer>("node_status_viewer")
                .unwrap();
            node_status_viewer.receive_messages();
        }
        {
            let status_bar = runner.find_name::<StatusBar>("status_bar").unwrap();
            let mut log_viewer_hideable = runner
                .find_name::<HideableView<BoxedView>>("log_viewer_hideable")
                .unwrap();
            let mut message_viewer_hideable = runner
                .find_name::<HideableView<BoxedView>>("message_viewer_hideable")
                .unwrap();
            let mut node_status_hideable = runner
                .find_name::<HideableView<BoxedView>>("node_status_hideable")
                .unwrap();

            match status_bar.selected_tab() {
                SelectedTab::LogViewer => {
                    log_viewer_hideable.set_visible(true);
                    message_viewer_hideable.set_visible(false);
                    node_status_hideable.set_visible(false);
                }
                SelectedTab::CanMessageViewer => {
                    log_viewer_hideable.set_visible(false);
                    message_viewer_hideable.set_visible(true);
                    node_status_hideable.set_visible(false);
                }
                SelectedTab::NodeStatus => {
                    log_viewer_hideable.set_visible(false);
                    message_viewer_hideable.set_visible(false);
                    node_status_hideable.set_visible(true);
                }
            }
        }

        runner.step();
    }
    enable_stdout_logging(true);

    stop_tx.send(()).ok();
    Ok(())
}

/// Write every log line to the `.log` file until the producer hangs up.
///
/// `Lagged` is recoverable and must be treated that way. A `while let Ok(..)`
/// here ends the loop on the first lag, flushes, and returns `Ok(())` — after
/// which the file silently stops growing for the rest of the session, with a
/// clean exit code and nothing in the log to say so. Lagging is not exotic:
/// every line costs this task several blocking-pool syscall round-trips
/// (`tokio::fs::File` does no userspace buffering) while the producer only has
/// to format and push, so a burst of target output outruns a 256-deep channel
/// easily. Only `Closed` ends the loop; a lag leaves a marker and continues.
async fn log_saver_task(
    mut log_saver: LogSaver,
    mut logs_rx: broadcast::Receiver<TargetLog>,
) -> Result<()> {
    loop {
        match logs_rx.recv().await {
            Ok(log) => log_saver.append_log(&log).await?,
            Err(broadcast::error::RecvError::Lagged(dropped)) => {
                log_saver.append_dropped_marker(dropped).await?
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
    log_saver.flush().await?;

    Ok(())
}

/// Same contract as [`log_saver_task`], for the `.message.log` / `.imu.csv`
/// pair. The CAN channel is only 32 deep — roughly eight iterations of the
/// serial read loop — so this is the one that lags first.
async fn message_saver_task(
    mut message_saver: CanMessageSaver,
    mut messages_rx: broadcast::Receiver<DecodedMessage>,
) -> Result<()> {
    loop {
        match messages_rx.recv().await {
            Ok(message) => message_saver.append_message(&message).await?,
            Err(broadcast::error::RecvError::Lagged(dropped)) => {
                message_saver.append_dropped_marker(dropped).await?
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
    message_saver.flush().await?;

    Ok(())
}
