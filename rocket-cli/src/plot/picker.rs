//! Choosing which flight to plot when the log holds more than one.
//!
//! Reached only when the CSV actually contains several sessions. A picker that
//! appeared for the common single-flight case would be a keystroke charged for
//! nothing, so [`super::plot_flight_log`] skips it.

use anyhow::{Result, bail};
use chrono::{Local, TimeZone};
use cursive::{
    Cursive,
    align::HAlign,
    theme::{Color, ColorStyle, Palette, PaletteStyle, Style},
    view::{Nameable, Resizable, Scrollable},
    views::{Dialog, LinearLayout, SelectView, TextView},
};
use std::sync::{Arc, Mutex};

use crate::plot::log_csv::FlightLog;
use crate::plot::session::{Session, WindowSource};

/// Wall-clock start of a session, where the log had a GPS-disciplined clock.
fn started_at(session: &Session) -> String {
    match session.unix_time_us {
        Some(us) => match Local.timestamp_micros(us as i64) {
            chrono::LocalResult::Single(t) => t.format("%Y-%m-%d %H:%M:%S").to_string(),
            _ => "unknown date     ".to_string(),
        },
        None => "no GPS time      ".to_string(),
    }
}

/// One row of the picker.
///
/// Carries every fact needed to tell two flights apart without opening them:
/// when, how long, how high, and whether the window is trustworthy.
pub fn describe(index: usize, session: &Session, log: &FlightLog) -> String {
    let apogee = match session.apogee_asl {
        Some(a) => format!("{a:>7.0} m"),
        None => "      —".to_string(),
    };
    let note = match session.window_source {
        WindowSource::Stages => "",
        WindowSource::StagesNoLanding => "  (no landing in log)",
        WindowSource::NeverLeftThePad => "  (never launched)",
    };
    format!(
        "{:>2}.  {}  {:>8.1} s   apogee {}   {:>8} rows{}",
        index + 1,
        started_at(session),
        session.duration_s(log),
        apogee,
        session.flight_rows(),
        note,
    )
}

/// Present the sessions and return the chosen index.
///
/// `None` means the user backed out, which is a normal outcome and not an
/// error — the caller exits quietly rather than rendering something unasked
/// for.
pub fn pick_session(sessions: &[Session], log: &FlightLog, source: &str) -> Result<Option<usize>> {
    if sessions.is_empty() {
        bail!("no sessions found");
    }

    // One channel for both the Enter path and the button path, so the two
    // cannot disagree about what was picked.
    let chosen: Arc<Mutex<Option<usize>>> = Arc::new(Mutex::new(None));

    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();
    theme.palette = Palette::terminal_default();
    theme.palette[PaletteStyle::View] =
        Style::from_color_style(ColorStyle::back(Color::Rgb(0x14, 0x17, 0x1F)));
    theme.palette[PaletteStyle::Primary] =
        Style::from_color_style(ColorStyle::front(Color::Rgb(0xD7, 0xDC, 0xE8)));
    siv.set_theme(theme);

    let mut select = SelectView::new().h_align(HAlign::Left);
    for (i, session) in sessions.iter().enumerate() {
        select.add_item(describe(i, session, log), i);
    }
    // Default to the most recent flight. Someone who just landed and pulled the
    // card wants the one they were standing next to, not the first of the day.
    let _ = select.set_selection(sessions.len() - 1);

    let on_submit = chosen.clone();
    select.set_on_submit(move |s: &mut Cursive, index: &usize| {
        *on_submit.lock().unwrap() = Some(*index);
        s.quit();
    });

    let on_button = chosen.clone();
    siv.add_layer(
        Dialog::around(
            LinearLayout::vertical()
                .child(TextView::new(
                    "↑/↓ to move · Enter to plot · Esc to cancel",
                ))
                .child(TextView::new(" "))
                .child(
                    select
                        .with_name("sessions")
                        .scrollable()
                        .min_height(5)
                        .max_height(20),
                ),
        )
        .title(format!("{} flights in {source}", sessions.len()))
        .padding_lrtb(2, 2, 1, 1)
        .button("Plot", move |s| {
            let index = s
                .find_name::<SelectView<usize>>("sessions")
                .and_then(|v| v.selection())
                .map(|rc| *rc);
            if let Some(index) = index {
                *on_button.lock().unwrap() = Some(index);
                s.quit();
            }
        })
        .button("Cancel", |s| s.quit()),
    );
    siv.add_global_callback(cursive::event::Key::Esc, |s| s.quit());

    siv.run();

    let picked = *chosen.lock().unwrap();
    Ok(picked)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plot::log_csv::test_support::log_from_csv;

    fn session(rows: usize, apogee: Option<f32>, source: WindowSource) -> Session {
        Session {
            start: 0,
            end: rows,
            flight_start: 0,
            flight_end: rows,
            window_source: source,
            unix_time_us: None,
            apogee_asl: apogee,
            apogee_row: None,
        }
    }

    /// A row must not print a number where the log had none — an apogee of `0 m`
    /// and an apogee that was never recorded are different flights.
    #[test]
    fn a_row_without_apogee_shows_a_dash_not_a_zero() {
        let log = log_from_csv(
            "picker_no_apogee",
            "timestamp_us\n0\n1000000\n",
        );
        let row = describe(0, &session(2, None, WindowSource::Stages), &log);
        assert!(row.contains('—'), "{row}");
        assert!(!row.contains("0 m"), "{row}");
    }

    /// A window the detector was not confident about has to say so here, where
    /// the choice is made, not only on the rendered image.
    #[test]
    fn an_untrustworthy_window_is_flagged_in_the_row() {
        let log = log_from_csv(
            "picker_never_launched",
            "timestamp_us\n0\n1000000\n",
        );
        let row = describe(
            1,
            &session(2, Some(1000.0), WindowSource::NeverLeftThePad),
            &log,
        );
        assert!(row.contains("never launched"), "{row}");
        assert!(row.starts_with(" 2."), "{row}");
    }
}
