pub mod log_saver;
mod logs_view;
pub mod target_log;

use std::sync::{Arc, RwLock};

use cursive::{
    View,
    event::EventResult,
    theme::Color,
    view::{Finder, Nameable as _, Resizable, ScrollStrategy, Scrollable as _, ViewWrapper},
    views::{Button, Checkbox, Dialog, EditView, LinearLayout, ListView, Panel, TextView},
    wrap_impl,
};
use log::Level;
use target_log::TargetLog;
use tokio::sync::broadcast;

use crate::monitor::log::logs_view::LogsView;

use super::config::MonitorConfig;

pub struct LogViewer {
    root: LinearLayout,
    logs_rx: Arc<RwLock<broadcast::Receiver<TargetLog>>>,
    paused: Arc<RwLock<bool>>,
}

impl LogViewer {
    pub fn new(
        config: Arc<RwLock<MonitorConfig>>,
        logs_rx: broadcast::Receiver<TargetLog>,
    ) -> Self {
        let paused = Arc::new(RwLock::new(false));
        let config_a = config.clone();
        let config_b = config.clone();
        let config_c = config.clone();
        let config_d = config.clone();
        Self {
            logs_rx: Arc::new(RwLock::new(logs_rx)),
            paused: paused.clone(),
            root: LinearLayout::vertical()
                .child(
                    LinearLayout::horizontal()
                        .child(
                            Button::new("Pause", move |siv| {
                                let mut paused_guard = paused.write().unwrap();
                                *paused_guard = !*paused_guard;

                                let mut pause_button =
                                    siv.find_name::<Button>("pause_button").unwrap();

                                if *paused_guard {
                                    pause_button.set_label("Paused");
                                } else {
                                    pause_button.set_label("Pause");
                                }
                            })
                            .with_name("pause_button")
                            .fixed_width(9),
                        )
                        .child(
                            Button::new("Filter", move |siv| {
                                siv.add_layer(Self::create_filter_dialog(config_a.clone()));
                            })
                            .fixed_width(9),
                        )
                        .child(TextView::new("Module: "))
                        .child(
                            EditView::new()
                                .content(&config_b.clone().read().unwrap().module)
                                .on_edit(move |_, text, __| {
                                    let config = config_b.clone();
                                    let mut config_guard = config.write().unwrap();
                                    config_guard.module = text.into();
                                    config_guard.save().ok();
                                })
                                .full_width()
                                .with_name("module"),
                        )
                        .child(TextView::new("  Search: "))
                        .child(
                            EditView::new()
                                .content(&config_c.clone().read().unwrap().search)
                                .on_edit(move |_, text, __| {
                                    let config = config_c.clone();
                                    let mut config_guard = config.write().unwrap();
                                    config_guard.search = text.into();
                                    config_guard.save().ok();

                                    // TODO
                                })
                                .full_width()
                                .with_name("search"),
                        ),
                )
                .child(
                    LogsView::new(config_d)
                        .with_name("logs")
                        .scrollable()
                        .scroll_strategy(ScrollStrategy::StickToBottom)
                        .on_scroll_inner(|v, _| {
                            if v.is_at_bottom() {
                                v.set_scroll_strategy(ScrollStrategy::StickToBottom);
                            }
                            EventResult::Consumed(None)
                        })
                        .with_name("logs_scroll_view"),
                ),
        }
    }

    fn create_filter_dialog(config: Arc<RwLock<MonitorConfig>>) -> Dialog {
        Dialog::around(
            LinearLayout::horizontal()
                .child(
                    Panel::new(
                        ListView::new()
                            .child(
                                "Trace",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.trace,
                                    |c, v| c.levels.trace = v,
                                ),
                            )
                            .child(
                                "Debug",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.debug,
                                    |c, v| c.levels.debug = v,
                                ),
                            )
                            .child(
                                "Info",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.info,
                                    |c, v| c.levels.info = v,
                                ),
                            )
                            .child(
                                "Warn",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.warn,
                                    |c, v| c.levels.warn = v,
                                ),
                            )
                            .child(
                                "Error",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.error,
                                    |c, v| c.levels.error = v,
                                ),
                            )
                            .child(
                                "Plain Text",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.levels.plain_text,
                                    |c, v| c.levels.plain_text = v,
                                ),
                            )
                            .scrollable(),
                    )
                    .title("Level"),
                )
                .child(
                    Panel::new(
                        ListView::new()
                            .child(
                                "Void Lake",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.void_lake,
                                    |c, v| c.devices.void_lake = v,
                                ),
                            )
                            .child(
                                "AMP",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.amp,
                                    |c, v| c.devices.amp = v,
                                ),
                            )
                            .child(
                                "AMP Speed Bridge",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.amp_speed_bridge,
                                    |c, v| c.devices.amp_speed_bridge = v,
                                ),
                            )
                            .child(
                                "ICARUS",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.icarus,
                                    |c, v| c.devices.icarus = v,
                                ),
                            )
                            .child(
                                "Payload Activation",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.payload_activation,
                                    |c, v| c.devices.payload_activation = v,
                                ),
                            )
                            .child(
                                "Rocket WiFi",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.rocket_wifi,
                                    |c, v| c.devices.rocket_wifi = v,
                                ),
                            )
                            .child(
                                "OZYS",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.ozys,
                                    |c, v| c.devices.ozys = v,
                                ),
                            )
                            .child(
                                "Bulkhead",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.bulkhead,
                                    |c, v| c.devices.bulkhead = v,
                                ),
                            )
                            .child(
                                "EPS 1",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.eps1,
                                    |c, v| c.devices.eps1 = v,
                                ),
                            )
                            .child(
                                "EPS 2",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.eps2,
                                    |c, v| c.devices.eps2 = v,
                                ),
                            )
                            .child(
                                "AeroRust",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.aerorust,
                                    |c, v| c.devices.aerorust = v,
                                ),
                            )
                            .child(
                                "Other",
                                Self::create_config_checkbox(
                                    config.clone(),
                                    |c| c.devices.other,
                                    |c, v| c.devices.other = v,
                                ),
                            )
                            .scrollable(),
                    )
                    .title("Device"),
                ),
        )
        .title("Filter")
        .button("OK", |siv| {
            siv.pop_layer();
            siv.focus_name("search").unwrap();
        })
    }

    fn create_config_checkbox(
        config: Arc<RwLock<MonitorConfig>>,
        get_value: impl Fn(&MonitorConfig) -> bool + Send + Sync + 'static,
        set_value: impl Fn(&mut MonitorConfig, bool) + Send + Sync + 'static,
    ) -> impl View {
        Checkbox::new()
            .with_checked(get_value(&config.read().unwrap()))
            .on_change({
                let config = config.clone();
                move |_, new_value| {
                    let mut config_guard = config.write().unwrap();
                    set_value(&mut config_guard, new_value);
                    config_guard.save().ok();
                }
            })
    }

    pub fn receive_logs(&mut self) {
        let mut logs_rx = self.logs_rx.write().unwrap();
        let mut logs_view = self.root.find_name::<LogsView>("logs").unwrap();
        while let Ok(log) = logs_rx.try_recv() {
            if !*self.paused.read().unwrap() {
                logs_view.push_log(log);
            }
        }
    }
}

impl ViewWrapper for LogViewer {
    wrap_impl!(self.root: LinearLayout);
}

pub fn log_level_foreground_color(log_level: Level) -> Color {
    match log_level {
        Level::Trace => Color::Rgb(127, 127, 127),
        Level::Debug => Color::Rgb(0, 0, 255),
        Level::Info => Color::Rgb(0, 160, 0),
        Level::Warn => Color::Rgb(127, 127, 0),
        Level::Error => Color::Rgb(255, 0, 0),
    }
}
