use std::sync::RwLock;

use cursive::{
    theme::{BaseColor, Color, Palette, PaletteColor},
    view::{Nameable as _, Resizable, Scrollable as _},
    views::{Button, EditView, LinearLayout, TextView},
};

fn main() {
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
            .child(TextView::new("").with_name("logs").scrollable()),
    );
    // ProgressBar

    siv.focus_name("filter").unwrap();
    siv.set_autorefresh(true);

    let mut count = 0;
    let mut runner = siv.runner();
    runner.refresh();
    while runner.is_running() {
        runner.step();
        count += 1;
        let mut logs_view = runner.find_name::<TextView>("logs").unwrap();
        logs_view.set_content(format!("{}", count));
    }
}
