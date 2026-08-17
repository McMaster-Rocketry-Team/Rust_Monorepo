//! `plot-flight-log`: turn a downloaded flight-log CSV into two 4K figures.
//!
//! The three things this has to get right, in the order a reader meets them:
//! pick the *right flight* out of a log that may hold several, cut it down to
//! the part where the rocket was actually in the air, and then draw it.

pub mod events;
pub mod figures;
pub mod log_csv;
pub mod picker;
pub mod series;
pub mod session;
pub mod theme;

use std::io::IsTerminal;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

use crate::args::PlotFlightLogArgs;
use figures::Renderer;
use log_csv::FlightLog;
use session::{Session, WindowSource, find_sessions};

pub fn plot_flight_log(args: &PlotFlightLogArgs) -> Result<()> {
    let input = Path::new(&args.input);
    let log = FlightLog::load(input)?;
    let sessions = find_sessions(&log, args.lead_in);

    let source_name = input
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| args.input.clone());

    println!(
        "{}: {} row(s), {} armed session(s)",
        source_name,
        log.row_count,
        sessions.len()
    );
    for (i, session) in sessions.iter().enumerate() {
        println!("  {}", picker::describe(i, session, &log));
    }
    if let Some(failed) = log.crc_failed_rows.filter(|n| *n > 0) {
        println!(
            "  note: {failed} row(s) came from blocks whose CRC did not match; \
             they are plotted but are not evidence on their own"
        );
    }

    let index = choose(&sessions, &log, &source_name, args)?;
    let Some(index) = index else {
        println!("Cancelled.");
        return Ok(());
    };
    let session = &sessions[index];

    report_window(session, &log);

    let (flight_path, misc_path) = output_paths(input, args, index, sessions.len())?;
    let renderer = Renderer::new(&log, session, source_name);

    renderer
        .render_flight(&flight_path)
        .with_context(|| format!("writing {}", flight_path.display()))?;
    renderer
        .render_misc(&misc_path)
        .with_context(|| format!("writing {}", misc_path.display()))?;

    println!(
        "Wrote {} and {} ({}×{})",
        flight_path.display(),
        misc_path.display(),
        figures::WIDTH,
        figures::HEIGHT
    );
    Ok(())
}

/// Decide which session to plot: the flag, the only one there is, or the picker.
fn choose(
    sessions: &[Session],
    log: &FlightLog,
    source_name: &str,
    args: &PlotFlightLogArgs,
) -> Result<Option<usize>> {
    if let Some(requested) = args.session {
        if requested == 0 || requested > sessions.len() {
            bail!(
                "--session {requested} is out of range; this log has {} session(s)",
                sessions.len()
            );
        }
        return Ok(Some(requested - 1));
    }
    if sessions.len() == 1 {
        return Ok(Some(0));
    }
    // The picker needs a terminal to draw on. Failing with the fix in the
    // message beats cursive's own error from inside a pipe or a CI job.
    if !std::io::stdin().is_terminal() || !std::io::stdout().is_terminal() {
        bail!(
            "this log holds {} sessions and there is no terminal to show the picker on; \
             pass --session <1..{}> to choose one",
            sessions.len(),
            sessions.len()
        );
    }
    picker::pick_session(sessions, log, source_name)
}

/// Say what was trimmed, on stdout as well as on the image.
///
/// The trim is the one thing here that silently discards data, so it is stated
/// every run rather than left for someone to infer from an axis that starts at
/// zero.
fn report_window(session: &Session, log: &FlightLog) {
    match session.window_source {
        WindowSource::Stages => println!(
            "Flight window: T-{:.0} s to landing, {:.1} s of flight — trimmed {:.1} s \
             on the pad and {:.1} s after landing.",
            session.lead_in_s(log),
            session.duration_s(log),
            session.trimmed_before_s(log),
            session.trimmed_after_s(log),
        ),
        WindowSource::StagesNoLanding => println!(
            "Flight window: T-{:.0} s to end of log, {:.1} s of flight — trimmed {:.1} s \
             on the pad. The log ends before landing.",
            session.lead_in_s(log),
            session.duration_s(log),
            session.trimmed_before_s(log),
        ),
        WindowSource::NeverLeftThePad => println!(
            "This session never left the pad — no ascent was ever logged. \
             Plotting all {:.1} s of it.",
            session.duration_s(log),
        ),
    }
}

/// Where the two PNGs go.
///
/// The session number only enters the filename when there was a choice to make,
/// so the ordinary one-flight case produces predictable names.
fn output_paths(
    input: &Path,
    args: &PlotFlightLogArgs,
    index: usize,
    session_count: usize,
) -> Result<(PathBuf, PathBuf)> {
    let stem = input
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "flight_log".to_string());
    let stem = if session_count > 1 {
        format!("{stem}_s{}", index + 1)
    } else {
        stem
    };

    let dir = match &args.out_dir {
        Some(dir) => PathBuf::from(dir),
        None => input.parent().unwrap_or(Path::new(".")).to_path_buf(),
    };
    if !dir.as_os_str().is_empty() {
        std::fs::create_dir_all(&dir)
            .with_context(|| format!("creating {}", dir.display()))?;
    }

    Ok((
        dir.join(format!("{stem}_flight.png")),
        dir.join(format!("{stem}_misc.png")),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(out_dir: Option<&str>) -> PlotFlightLogArgs {
        PlotFlightLogArgs {
            input: "flight_log.csv".to_string(),
            out_dir: out_dir.map(str::to_string),
            session: None,
            lead_in: 0.0,
        }
    }

    /// A single-flight log gets clean names; a multi-flight log gets names that
    /// cannot overwrite each other when the other flights are plotted next.
    #[test]
    fn the_session_number_only_enters_the_name_when_there_was_a_choice() {
        let dir = std::env::temp_dir();
        let input = dir.join("hil_dual_2026-08-17.csv");

        let (flight, misc) = output_paths(&input, &args(None), 0, 1).unwrap();
        assert_eq!(flight.file_name().unwrap(), "hil_dual_2026-08-17_flight.png");
        assert_eq!(misc.file_name().unwrap(), "hil_dual_2026-08-17_misc.png");

        let (flight, _) = output_paths(&input, &args(None), 1, 3).unwrap();
        assert_eq!(
            flight.file_name().unwrap(),
            "hil_dual_2026-08-17_s2_flight.png"
        );
    }

    /// Images land next to the CSV unless told otherwise — the common case is
    /// plotting a log you just downloaded into the directory you are standing in.
    #[test]
    fn images_default_to_the_csvs_own_directory() {
        let input = Path::new("/tmp/some/where/flight_log.csv");
        let (flight, _) = output_paths(input, &args(None), 0, 1).unwrap();
        assert_eq!(flight.parent().unwrap(), Path::new("/tmp/some/where"));
    }

    #[test]
    fn an_out_of_range_session_flag_is_rejected_with_the_count() {
        let log = log_csv::test_support::log_from_csv(
            "mod_range",
            "record_count,timestamp_us,flight_stage\n0,0,Ascent\n1,1000,Landed\n",
        );
        let sessions = find_sessions(&log, 0.0);
        let mut a = args(None);
        a.session = Some(4);
        let err = choose(&sessions, &log, "x.csv", &a).unwrap_err().to_string();
        assert!(err.contains("out of range"), "{err}");
        assert!(err.contains("1 session"), "{err}");
    }

    /// `--session` is 1-based on the command line because the listing it mirrors
    /// is 1-based.
    #[test]
    fn the_session_flag_is_one_based() {
        let log = log_csv::test_support::log_from_csv(
            "mod_one_based",
            "record_count,timestamp_us,flight_stage\n\
             0,0,Ascent\n1,1000,Landed\n0,2000,Ascent\n1,3000,Landed\n",
        );
        let sessions = find_sessions(&log, 0.0);
        assert_eq!(sessions.len(), 2);
        let mut a = args(None);
        a.session = Some(2);
        assert_eq!(choose(&sessions, &log, "x.csv", &a).unwrap(), Some(1));
    }
}
