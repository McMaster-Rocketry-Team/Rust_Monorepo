pub mod log_saver;
pub mod target_log;

use std::sync::{Arc, RwLock};

use anyhow::Result;
use cursive::{
    Printer, Rect, Vec2, View,
    direction::Direction,
    event::{Callback, Event, EventResult, MouseButton, MouseEvent},
    theme::{Color, ColorStyle, Effects, Style},
    utils::markup::StyledString,
    view::{
        CannotFocus, Finder, Nameable as _, Resizable, ScrollStrategy, Scrollable as _,
        ViewWrapper, scroll,
    },
    views::{
        Button, Checkbox, Dialog, EditView, LinearLayout, ListView, NamedView, Panel, ScrollView,
        TextView,
    },
    wrap_impl,
};
use log::Level;
use pad::PadStr;
use target_log::{DefmtLogInfo, TargetLog};
use tokio::sync::broadcast;

use super::config::MonitorConfig;

pub struct LogViewer {
    root: LinearLayout,
    logs_rx: Arc<RwLock<broadcast::Receiver<TargetLog>>>,
    paused: Arc<RwLock<bool>>,
    config: Arc<RwLock<MonitorConfig>>,
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
        Self {
            logs_rx: Arc::new(RwLock::new(logs_rx)),
            paused: paused.clone(),
            config,
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
                        .child(TextView::new("Search: "))
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
                    LinearLayout::vertical()
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
        let mut logs_view = self.root.find_name::<LinearLayout>("logs").unwrap();
        while let Ok(log) = logs_rx.try_recv() {
            if !*self.paused.read().unwrap() {
                logs_view.add_child(LogRow::new(log, self.config.clone()));
                while logs_view.len() > 500 {
                    logs_view.remove_child(0);
                }
            }
        }
    }
}

impl ViewWrapper for LogViewer {
    wrap_impl!(self.root: LinearLayout);
}

struct LogRow {
    log: TargetLog,
    last_size: Vec2,
    show_line_number: bool,
    config: Arc<RwLock<MonitorConfig>>,
    matches: bool,
    log_content_offset: usize,
}

impl LogRow {
    pub fn new(log: TargetLog, config: Arc<RwLock<MonitorConfig>>) -> Self {
        return Self {
            log_content_offset: if log.defmt.is_some() { 23 } else { 8 },
            log,
            last_size: Vec2::zero(),
            show_line_number: false,
            config,
            matches: false,
        };
    }
}

impl View for LogRow {
    fn draw(&self, printer: &Printer) {
        if !self.matches {
            return;
        }

        let bg = self.log.node_type.background_color();

        printer.with_color(ColorStyle::new(Color::Rgb(0, 0, 0), bg), |printer| {
            for y in 0..printer.size.y {
                printer.print_hline((0, y), printer.size.x, " ");
            }

            printer.print((0, 0), &self.log.node_type.short_name().pad_to_width(4));
            printer.print(
                (4, 0),
                &self.log.node_id.map_or(String::from("xxx "), |id| {
                    format!("{:X} ", id).pad(4, '0', pad::Alignment::Right, false)
                }),
            );

            if let Some(defmt_info) = &self.log.defmt {
                printer.print_styled(
                    (8, 0),
                    &StyledString::single_span(
                        defmt_info.log_level.to_string().pad_to_width(6),
                        Style::from_color_style(ColorStyle::front(log_level_foreground_color(
                            defmt_info.log_level,
                        ))),
                    ),
                );
                let timestamp = defmt_info
                    .timestamp
                    .map_or(String::new(), |t| format!("{:.3}", t))
                    .pad_to_width_with_alignment(8, pad::Alignment::Right)
                    .pad_to_width(9);
                printer.print_styled(
                    (14, 0),
                    &StyledString::single_span(
                        timestamp,
                        Style::from_color_style(ColorStyle::front(Color::Rgb(100, 100, 100))),
                    ),
                );
            }

            if printer.size.x > self.log_content_offset {
                let log_content_width = printer.size.x - self.log_content_offset;
                let mut i = 0;
                for y in 0..(printer.size.y - if self.show_line_number { 1 } else { 0 }) {
                    printer.print(
                        (self.log_content_offset, y),
                        &self.log.log_content
                            [i..(i + log_content_width).min(self.log.log_content.len())],
                    );
                    i += log_content_width;
                }
            }

            if self.show_line_number {
                if let Some(DefmtLogInfo {
                    location: Some(location),
                    ..
                }) = &self.log.defmt
                {
                    printer.print(
                        (0, printer.size.y - 1),
                        &format!(
                            "└─ {} @ {}:{}",
                            location.module_path, location.file_path, location.line_number
                        ),
                    );
                } else {
                    printer.print((0, printer.size.y - 1), "└─ Line number info not avaliable");
                }
            }
        });
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        self.matches = self.config.read().unwrap().matches(&self.log);
        if !self.matches {
            return Vec2::zero();
        }

        if constraint.x <= self.log_content_offset || self.log.log_content.len() == 0 {
            return Vec2 {
                x: constraint.x,
                y: 1,
            };
        }

        let log_content_width = constraint.x - self.log_content_offset;
        let log_content_lines = (self.log.log_content.len() - 1) / log_content_width + 1;

        return Vec2 {
            x: constraint.x,
            y: log_content_lines + if self.show_line_number { 1 } else { 0 },
        };
    }

    fn layout(&mut self, size: Vec2) {
        self.last_size = size;
    }

    fn on_event(&mut self, event: Event) -> EventResult {
        if !self.matches {
            return EventResult::Ignored;
        }
        match event {
            Event::Mouse {
                event: mouse_event,
                position,
                offset,
            } if position.fits_in_rect(offset, self.last_size) => {
                if mouse_event == MouseEvent::Release(MouseButton::Left) {
                    self.show_line_number = !self.show_line_number;
                    EventResult::Consumed(None)
                } else if mouse_event == MouseEvent::WheelUp || mouse_event == MouseEvent::WheelDown
                {
                    EventResult::Consumed(Some(Callback::from_fn(move |s| {
                        let mut logs_scroll_view = s
                            .find_name::<ScrollView<NamedView<LinearLayout>>>("logs_scroll_view")
                            .unwrap();

                        scroll::on_event(
                            &mut *logs_scroll_view,
                            event.clone(),
                            |_, __| EventResult::Ignored,
                            |_, __| Rect::from_point(Vec2::zero()),
                        );
                    })))
                } else {
                    EventResult::Ignored
                }
            }
            _ => EventResult::Ignored,
        }
    }

    fn take_focus(&mut self, _: Direction) -> Result<EventResult, CannotFocus> {
        Ok(EventResult::Consumed(None))
    }
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
