use std::{
    sync::RwLock,
    time::Duration,
};

use crate::target_log::{TargetLog, log_level_foreground_color};
use cursive::{
    Printer, Rect, Vec2, View,
    direction::Direction,
    event::{Callback, Event, EventResult, MouseButton, MouseEvent},
    theme::{Color, ColorStyle, Effect, Effects, Palette, Style},
    utils::markup::StyledString,
    view::{
        CannotFocus, Nameable as _, Resizable, ScrollStrategy, Scrollable as _, scroll,
    },
    views::{
        Button, Checkbox, Dialog, EditView, LinearLayout, ListView, NamedView, Panel,
        ScrollView, TextView,
    },
};
use pad::PadStr;
use tokio::{sync::broadcast, time};

pub async fn log_viewer_tui(mut logs_rx: broadcast::Receiver<TargetLog>) {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();

    theme.palette = Palette::terminal_default();
    siv.set_theme(theme);

    let paused = RwLock::new(false);
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
                            siv.add_layer(
                                Dialog::around(
                                    LinearLayout::horizontal()
                                        .child(
                                            Panel::new(
                                                ListView::new()
                                                    .child("Trace", Checkbox::new())
                                                    .child("Debug", Checkbox::new())
                                                    .child("Info", Checkbox::new())
                                                    .child("Warn", Checkbox::new())
                                                    .child("Error", Checkbox::new())
                                                    .scrollable(),
                                            )
                                            .title("Level"),
                                        )
                                        .child(
                                            Panel::new(
                                                ListView::new()
                                                    .child("Void Lake", Checkbox::new())
                                                    .child("AMP", Checkbox::new())
                                                    .child("ICARUS", Checkbox::new())
                                                    .child("Payload Activation", Checkbox::new())
                                                    .child("Rocket WiFi", Checkbox::new())
                                                    .child("OZYS", Checkbox::new())
                                                    .child("Bulkhead", Checkbox::new())
                                                    .child("EPS 1", Checkbox::new())
                                                    .child("EPS 2", Checkbox::new())
                                                    .child("AeroRust", Checkbox::new())
                                                    .scrollable(),
                                            )
                                            .title("Device"),
                                        ),
                                )
                                .title("Filter")
                                .button("OK", |siv| {
                                    siv.pop_layer();
                                    siv.focus_name("search").unwrap();
                                }),
                            );
                        })
                        .with_name("pause_button")
                        .fixed_width(9),
                    )
                    .child(TextView::new("Search: "))
                    .child(
                        EditView::new()
                            .on_edit(|_, __, ___| {
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

    siv.focus_name("search").unwrap();
    siv.set_autorefresh(true);

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));
    while runner.is_running() {
        runner.step();

        while let Ok(log) = logs_rx.try_recv() {
            let mut logs_view = runner.find_name::<LinearLayout>("logs").unwrap();
            logs_view.add_child(LogRow::new(log));
        }

        interval.tick().await;
    }
}

struct LogRow {
    log: TargetLog,
    last_size: Vec2,
    show_line_number: bool,
}

impl LogRow {
    pub fn new(log: TargetLog) -> Self {
        return Self {
            log,
            last_size: Vec2::zero(),
            show_line_number: false,
        };
    }
}

impl View for LogRow {
    fn draw(&self, printer: &Printer) {
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
                        effects: Effects::only(Effect::Bold),
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
