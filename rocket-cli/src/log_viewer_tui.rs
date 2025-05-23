use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use crate::{
    config::LogViewerConfig,
    target_log::{TargetLog, log_level_foreground_color},
};
use anyhow::Result;
use cursive::{
    Printer, Rect, Vec2, View,
    direction::Direction,
    event::{Callback, Event, EventResult, MouseButton, MouseEvent},
    theme::{Color, ColorStyle, Effects, Palette, Style},
    utils::markup::StyledString,
    view::{CannotFocus, Nameable as _, Resizable, ScrollStrategy, Scrollable as _, scroll},
    views::{
        Button, Checkbox, Dialog, EditView, LinearLayout, ListView, NamedView, Panel, ScrollView,
        TextView,
    },
};
use pad::PadStr;
use tokio::{sync::broadcast, time};

pub async fn log_viewer_tui(mut logs_rx: broadcast::Receiver<TargetLog>) -> Result<()> {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();

    theme.palette = Palette::terminal_default();
    siv.set_theme(theme);

    let paused = RwLock::new(false);
    let first_time = !LogViewerConfig::exists();
    let config = Arc::new(RwLock::new(LogViewerConfig::load()?));
    let config_a = config.clone();
    let config_b = config.clone();
    let config_c = config.clone();

    siv.add_fullscreen_layer(
        LinearLayout::vertical()
            .child(
                LinearLayout::horizontal()
                    .child(
                        Button::new("Pause", move |siv| {
                            siv.focus_name("search").unwrap();
                            let mut paused_guard = paused.write().unwrap();
                            *paused_guard = !*paused_guard;

                            let mut pause_button = siv.find_name::<Button>("pause_button").unwrap();

                            if *paused_guard {
                                pause_button.set_label("Paused");
                            } else {
                                pause_button.set_label("Pause");
                            }

                            // TODO
                        })
                        .with_name("pause_button")
                        .fixed_width(9),
                    )
                    .child(
                        Button::new("Filter", move |siv| {
                            siv.add_layer(create_filter_dialog(config_a.clone()));
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

                                // TODO
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
    );

    siv.set_autorefresh(true);

    if first_time {
        siv.add_layer(
            Dialog::around(TextView::new("Click on the log to view the line number"))
                .title("Tips")
                .button("OK", |s| {
                    s.pop_layer().unwrap();
                }),
        );
    } else {
        siv.focus_name("search").unwrap();
    }

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));
    while runner.is_running() {
        runner.step();

        while let Ok(log) = logs_rx.try_recv() {
            let mut logs_view = runner.find_name::<LinearLayout>("logs").unwrap();
            logs_view.add_child(LogRow::new(log, config.clone()));
            while logs_view.len() > 500 {
                logs_view.remove_child(0);
            }
        }

        interval.tick().await;
    }

    Ok(())
}

fn create_filter_dialog(config: Arc<RwLock<LogViewerConfig>>) -> Dialog {
    Dialog::around(
        LinearLayout::horizontal()
            .child(
                Panel::new(
                    ListView::new()
                        .child(
                            "Trace",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.levels.trace,
                                |c, v| c.levels.trace = v,
                            ),
                        )
                        .child(
                            "Debug",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.levels.debug,
                                |c, v| c.levels.debug = v,
                            ),
                        )
                        .child(
                            "Info",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.levels.info,
                                |c, v| c.levels.info = v,
                            ),
                        )
                        .child(
                            "Warn",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.levels.warn,
                                |c, v| c.levels.warn = v,
                            ),
                        )
                        .child(
                            "Error",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.levels.error,
                                |c, v| c.levels.error = v,
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
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.void_lake,
                                |c, v| c.devices.void_lake = v,
                            ),
                        )
                        .child(
                            "AMP",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.amp,
                                |c, v| c.devices.amp = v,
                            ),
                        )
                        .child(
                            "ICARUS",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.icarus,
                                |c, v| c.devices.icarus = v,
                            ),
                        )
                        .child(
                            "Payload Activation",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.payload_activation,
                                |c, v| c.devices.payload_activation = v,
                            ),
                        )
                        .child(
                            "Rocket WiFi",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.rocket_wifi,
                                |c, v| c.devices.rocket_wifi = v,
                            ),
                        )
                        .child(
                            "OZYS",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.ozys,
                                |c, v| c.devices.ozys = v,
                            ),
                        )
                        .child(
                            "Bulkhead",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.bulkhead,
                                |c, v| c.devices.bulkhead = v,
                            ),
                        )
                        .child(
                            "EPS 1",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.eps1,
                                |c, v| c.devices.eps1 = v,
                            ),
                        )
                        .child(
                            "EPS 2",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.eps2,
                                |c, v| c.devices.eps2 = v,
                            ),
                        )
                        .child(
                            "AeroRust",
                            create_config_checkbox(
                                config.clone(),
                                |c| c.devices.aerorust,
                                |c, v| c.devices.aerorust = v,
                            ),
                        )
                        .child(
                            "Other",
                            create_config_checkbox(
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
    config: Arc<RwLock<LogViewerConfig>>,
    get_value: impl Fn(&LogViewerConfig) -> bool + Send + Sync + 'static,
    set_value: impl Fn(&mut LogViewerConfig, bool) + Send + Sync + 'static,
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

struct LogRow {
    log: TargetLog,
    last_size: Vec2,
    show_line_number: bool,
    config: Arc<RwLock<LogViewerConfig>>,
    matches: bool,
}

impl LogRow {
    pub fn new(log: TargetLog, config: Arc<RwLock<LogViewerConfig>>) -> Self {
        return Self {
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
            printer.print_styled(
                (8, 0),
                &StyledString::single_span(
                    self.log.log_level.to_string().pad_to_width(6),
                    Style {
                        effects: Effects::default(),
                        color: ColorStyle::new(log_level_foreground_color(self.log.log_level), bg),
                    },
                ),
            );
            let timestamp = self
                .log
                .timestamp
                .map_or(String::new(), |t| format!("{:.2}", t))
                .pad_to_width_with_alignment(7, pad::Alignment::Right)
                .pad_to_width(8);
            printer.print_styled(
                (14, 0),
                &StyledString::single_span(
                    timestamp,
                    Style {
                        effects: Effects::default(),
                        color: ColorStyle::new(Color::Rgb(100, 100, 100), bg),
                    },
                ),
            );

            if printer.size.x > 22 {
                let log_content_width = printer.size.x - 22;
                let mut i = 0;
                for y in 0..(printer.size.y - if self.show_line_number { 1 } else { 0 }) {
                    printer.print(
                        (22, y),
                        &self.log.log_content
                            [i..(i + log_content_width).min(self.log.log_content.len())],
                    );
                    i += log_content_width;
                }
            }

            if self.show_line_number {
                printer.print(
                    (0, printer.size.y - 1),
                    &format!(
                        "└─ {} @ {}:{}",
                        self.log.module_path, self.log.file_path, self.log.line_number
                    ),
                );
            }
        });
    }

    fn required_size(&mut self, constraint: cursive::Vec2) -> cursive::Vec2 {
        self.matches = self.config.read().unwrap().matches(&self.log);
        if !self.matches {
            return Vec2::zero();
        }

        if constraint.x <= 22 || self.log.log_content.len() == 0 {
            return Vec2 {
                x: constraint.x,
                y: 1,
            };
        }

        let log_content_width = constraint.x - 22;
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
