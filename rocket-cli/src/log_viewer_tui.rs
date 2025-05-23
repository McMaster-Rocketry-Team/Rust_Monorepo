use std::{sync::RwLock, time::Duration};

use cursive::{
    Printer, View, inner_getters,
    theme::{BaseColor, Color, ColorStyle, Effect, Effects, Palette, PaletteColor, Style},
    utils::markup::StyledString,
    view::{Nameable as _, Resizable, ScrollStrategy, Scrollable as _, ViewWrapper},
    views::{Button, EditView, LinearLayout, TextView},
    wrap_impl,
};
use tokio::{sync::broadcast, time};

use crate::target_log::{TargetLog, log_level_foreground_color};

pub async fn log_viewer_tui(mut logs_rx: broadcast::Receiver<TargetLog>) {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();

    theme.palette = Palette::terminal_default();
    theme.palette[PaletteColor::Highlight] = Color::Light(BaseColor::White);
    siv.set_theme(theme);

    let paused = RwLock::new(false);
    siv.add_fullscreen_layer(
        LinearLayout::vertical()
            .child(
                LinearLayout::horizontal()
                    .child(
                        Button::new("Pause", move |siv| {
                            siv.focus_name("filter").unwrap();
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
                    .child(TextView::new("Filter: "))
                    .child(
                        EditView::new()
                            .on_edit(|_, __, ___| {
                                // TODO
                            })
                            .full_width()
                            .with_name("filter"),
                    ),
            )
            .child(
                LinearLayout::vertical()
                    .with_name("logs")
                    .scrollable()
                    .scroll_strategy(ScrollStrategy::StickToBottom),
            ),
    );

    siv.focus_name("filter").unwrap();
    siv.set_autorefresh(true);

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));
    while runner.is_running() {
        runner.step();

        while let Ok(log) = logs_rx.try_recv() {
            let color_style =
                ColorStyle::new(Color::Rgb(0, 0, 0), log.node_type.background_color());

            let mut layout = LinearLayout::horizontal()
                .child(
                    TextView::new(StyledString::single_span(
                        log.node_type.short_name(),
                        Style {
                            effects: Default::default(),
                            color: color_style,
                        },
                    ))
                    .fixed_width(4),
                )
                .child(
                    TextView::new(StyledString::single_span(
                        log.log_level.to_string(),
                        Style {
                            effects: Effects::only(Effect::Bold),
                            color: ColorStyle::new(
                                log_level_foreground_color(log.log_level),
                                log.node_type.background_color(),
                            ),
                        },
                    ))
                    .fixed_width(6),
                );

            if let Some(timestamp) = log.timestamp.map(|t| format!("{:>7.2}", t)) {
                layout.add_child(
                    TextView::new(StyledString::single_span(
                        &timestamp,
                        Style {
                            effects: Default::default(),
                            color: color_style,
                        },
                    ))
                    .fixed_width(timestamp.len() + 1),
                );
            }
            layout.add_child(
                TextView::new(StyledString::single_span(
                    log.log_content,
                    Style {
                        effects: Default::default(),
                        color: color_style,
                    },
                ))
                .full_width(),
            );

            let mut logs_view = runner.find_name::<LinearLayout>("logs").unwrap();
            logs_view.add_child(layout.with_color(color_style));
        }

        interval.tick().await;
    }
}

/** Fill a region with an arbitrary color. */
#[derive(Debug)]
pub struct ColoredLayer<T: View> {
    color: ColorStyle,
    view: T,
}

impl<T: View> ColoredLayer<T> {
    /// Wraps the given view.
    pub fn new(color: ColorStyle, view: T) -> Self {
        ColoredLayer { color, view }
    }

    inner_getters!(self.view: T);
}

impl<T: View> ViewWrapper for ColoredLayer<T> {
    wrap_impl!(self.view: T);

    fn wrap_draw(&self, printer: &Printer<'_, '_>) {
        printer.with_color(self.color, |printer| {
            for y in 0..printer.size.y {
                printer.print_hline((0, y), printer.size.x, " ");
            }
        });
        self.view.draw(printer);
    }
}

pub trait Colorable: View + Sized {
    /// Wraps `self` in a `ScrollView`.
    fn with_color(self, color: ColorStyle) -> ColoredLayer<Self> {
        ColoredLayer::new(color, self)
    }
}

impl<T: View> Colorable for T {}
