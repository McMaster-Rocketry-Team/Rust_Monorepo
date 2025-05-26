mod config;
mod log;
mod message;
mod status_bar;

use config::MonitorConfig;
use cursive::{
    theme::{Palette, PaletteStyle},
    view::{Nameable, Resizable},
    views::{BoxedView, Dialog, HideableView, LinearLayout, TextView},
};
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
pub use log::target_log;
use log::{LogViewer, log_saver::LogSaver};
use status_bar::{SelectedTab, StatusBar};
use tokio::{
    sync::{broadcast, oneshot, watch},
    time,
};

use crate::{args::NodeTypeEnum, connection_method::ConnectionMethod};
use anyhow::Result;
use std::{
    path::PathBuf,
    sync::{Arc, RwLock},
    time::Duration,
};

use self::target_log::TargetLog;

#[derive(Debug, Clone, Copy)]
pub enum MonitorStatus {
    Initialize,
    Normal,
    ChunkError,
    Overrun,
}

pub async fn monitor_tui(
    connection_method: &mut Box<dyn ConnectionMethod>,
    chip: &String,
    secret_path: &PathBuf,
    node_type: &NodeTypeEnum,
    firmware_elf_path: &PathBuf,
) -> Result<()> {
    let log_saver = LogSaver::new(
        firmware_elf_path
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string(),
        connection_method,
    )
    .await?;

    let (status_tx, status_rx) = watch::channel(MonitorStatus::Initialize);
    let (logs_tx, logs_rx) = broadcast::channel::<TargetLog>(256);
    let logs_rx2 = logs_tx.subscribe();
    let (messages_tx, messages_rx) = broadcast::channel::<DecodedMessage>(32);
    let (stop_tx, stop_rx) = oneshot::channel::<()>();

    let tui_future = tui_task(connection_method.name(), status_rx, logs_rx, stop_tx);

    let attach_future = connection_method.attach(
        chip,
        secret_path,
        node_type,
        firmware_elf_path,
        status_tx,
        logs_tx,
        messages_tx,
        stop_rx,
    );

    let log_saver_future = log_saver_task(log_saver, logs_rx2);

    let (attach_result, tui_result, log_saver_result) =
        tokio::join!(attach_future, tui_future, log_saver_future);
    attach_result?;
    tui_result?;
    log_saver_result?;

    Ok(())
}

async fn tui_task(
    connection_method_name: String,
    mut status_rx: watch::Receiver<MonitorStatus>,
    logs_rx: broadcast::Receiver<TargetLog>,
    stop_tx: oneshot::Sender<()>,
) -> Result<()> {
    let first_time = !MonitorConfig::exists();
    let config = Arc::new(RwLock::new(MonitorConfig::load()?));

    status_rx.changed().await?;

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
                HideableView::new(BoxedView::new(Box::new(
                    LogViewer::new(config, logs_rx)
                        .with_name("log_viewer")
                        .full_screen(),
                )))
                .visible(true)
                .with_name("log_viewer_hideable"),
            )
            .child(
                HideableView::new(BoxedView::new(Box::new(
                    TextView::new("Messages").full_screen(),
                )))
                .visible(false)
                .with_name("message_viewer_hideable"),
            )
            .child(
                HideableView::new(BoxedView::new(Box::new(
                    TextView::new("Node Status").full_screen(),
                )))
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

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));

    while runner.is_running() {
        {
            let mut log_view = runner.find_name::<LogViewer>("log_viewer").unwrap();
            log_view.receive_logs();

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
        interval.tick().await;
    }

    stop_tx.send(()).ok();
    Ok(())
}

async fn log_saver_task(
    mut log_saver: LogSaver,
    mut logs_rx: broadcast::Receiver<TargetLog>,
) -> Result<()> {
    while let Ok(log) = logs_rx.recv().await {
        log_saver.append_log(&log).await.unwrap();
    }
    log_saver.flush().await.unwrap();

    Ok(())
}
