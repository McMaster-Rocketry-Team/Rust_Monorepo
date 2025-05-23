use std::{sync::RwLock, time::Duration};

use cursive::{
    theme::{BaseColor, Color, Palette, PaletteColor},
    view::{Nameable as _, Resizable, ScrollStrategy, Scrollable as _},
    views::{Button, EditView, LinearLayout, NamedView, ScrollView, TextView},
};
use tokio::{sync::broadcast, time};

use crate::target_log::TargetLog;

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
                TextView::new("")
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
            let mut logs_view = runner.find_name::<TextView>("logs").unwrap();
            logs_view.append(format!("{}\n", log.log_content));
        }

        interval.tick().await;
    }
}
