//! `plot-flight-log`: turn a downloaded flight-log CSV into four 4K figures.
//!
//! The three things this has to get right, in the order a reader meets them:
//! pick the *right flight* out of a log that may hold several, cut it down to
//! the part where the rocket was actually in the air, and then draw it.
//!
//! The four figures split by *which part of the stack is being asked about*,
//! and they do not share a time window. The air-brakes figure ends at apogee,
//! because the estimator behind every trace on it is retired there and the
//! descent would be one long gap. The deployment, auxiliary and payload
//! figures run to landing, because that is where the deployment estimator, the
//! pyros and the payload's own experiments do their work.
//!
//! The payload gets a figure of its own rather than a corner of the auxiliary
//! one: its rails, actuators, load cells and per-channel state machines are
//! most of a figure's worth of columns by themselves, and sharing left both
//! halves narrower than either could be read at.
//!
//! Altitudes are drawn AGL throughout, converted from the ASL the log stores
//! using the pad reference it now carries. AGL is the unit every threshold in
//! the firmware is configured in — drogue minimum, main altitude, apogee
//! target — so it is the unit in which the figures can be checked against the
//! flight plan.

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
use figures::{Renderer, Window};
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

    let paths = output_paths(input, args, index, sessions.len())?;
    let to_landing = Window {
        start: session.plot_start,
        end: session.flight_end,
    };
    let to_apogee = airbrakes_window(session);

    // One renderer per figure: the window is what makes them different, and it
    // is baked in at construction so no drawing code has to be trusted to stay
    // inside a range it was told about separately.
    Renderer::new(&log, session, source_name.clone(), to_apogee)
        .render_airbrakes(&paths.airbrakes)
        .with_context(|| format!("writing {}", paths.airbrakes.display()))?;
    Renderer::new(&log, session, source_name.clone(), to_landing)
        .render_deployment(&paths.deployment)
        .with_context(|| format!("writing {}", paths.deployment.display()))?;
    Renderer::new(&log, session, source_name.clone(), to_landing)
        .render_misc(&paths.misc)
        .with_context(|| format!("writing {}", paths.misc.display()))?;
    Renderer::new(&log, session, source_name, to_landing)
        .render_payload(&paths.payload)
        .with_context(|| format!("writing {}", paths.payload.display()))?;

    println!(
        "Wrote {}, {}, {} and {} ({}×{})",
        paths.airbrakes.display(),
        paths.deployment.display(),
        paths.misc.display(),
        paths.payload.display(),
        figures::WIDTH,
        figures::HEIGHT
    );
    Ok(())
}

/// The air-brakes figure's window: the same lead-in, ending at apogee.
///
/// Apogee is included rather than cut just before it — it is the moment the
/// whole figure is about, and an axis that stops one row short of it would hide
/// the vertical velocity crossing zero.
///
/// Falls back to the full flight window when no apogee was recorded, which is
/// the never-left-the-pad case and any log that ends mid-ascent. Drawing the
/// whole session there is right: there is no apogee to stop at, and the reader
/// is looking at the figure precisely because something went wrong.
fn airbrakes_window(session: &Session) -> Window {
    let end = session
        .apogee_row
        .map(|row| (row + 1).clamp(session.plot_start + 1, session.flight_end))
        .unwrap_or(session.flight_end);
    Window {
        start: session.plot_start,
        end,
    }
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

/// Where the four PNGs go.
struct OutputPaths {
    airbrakes: PathBuf,
    deployment: PathBuf,
    misc: PathBuf,
    payload: PathBuf,
}

/// Name the four PNGs.
///
/// The session number only enters the filename when there was a choice to make,
/// so the ordinary one-flight case produces predictable names.
fn output_paths(
    input: &Path,
    args: &PlotFlightLogArgs,
    index: usize,
    session_count: usize,
) -> Result<OutputPaths> {
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

    Ok(OutputPaths {
        airbrakes: dir.join(format!("{stem}_airbrakes.png")),
        deployment: dir.join(format!("{stem}_deployment.png")),
        misc: dir.join(format!("{stem}_misc.png")),
        payload: dir.join(format!("{stem}_payload.png")),
    })
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

        let p = output_paths(&input, &args(None), 0, 1).unwrap();
        assert_eq!(
            p.airbrakes.file_name().unwrap(),
            "hil_dual_2026-08-17_airbrakes.png"
        );
        assert_eq!(
            p.deployment.file_name().unwrap(),
            "hil_dual_2026-08-17_deployment.png"
        );
        assert_eq!(p.misc.file_name().unwrap(), "hil_dual_2026-08-17_misc.png");
        assert_eq!(
            p.payload.file_name().unwrap(),
            "hil_dual_2026-08-17_payload.png"
        );

        let p = output_paths(&input, &args(None), 1, 3).unwrap();
        assert_eq!(
            p.airbrakes.file_name().unwrap(),
            "hil_dual_2026-08-17_s2_airbrakes.png"
        );
    }

    /// Images land next to the CSV unless told otherwise — the common case is
    /// plotting a log you just downloaded into the directory you are standing in.
    #[test]
    fn images_default_to_the_csvs_own_directory() {
        let input = Path::new("/tmp/some/where/flight_log.csv");
        let p = output_paths(input, &args(None), 0, 1).unwrap();
        assert_eq!(p.airbrakes.parent().unwrap(), Path::new("/tmp/some/where"));
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

    /// The air-brakes figure stops at apogee — inclusive, because apogee is the
    /// moment the figure is about and a window that ended one row short of it
    /// would cut the vertical velocity crossing zero.
    #[test]
    fn the_airbrakes_window_ends_on_apogee_not_before_it() {
        let log = log_csv::test_support::log_from_csv(
            "ab_window",
            concat!(
                "record_count,timestamp_us,flight_stage,deployment_kf_altitude_asl\n",
                "0,0,Armed,100\n",
                "1,1000000,Ascent,200\n",
                "2,2000000,Ascent,900\n",
                "3,3000000,DrogueChute,600\n",
                "4,4000000,DrogueChute,400\n",
                "5,5000000,MainChute,200\n",
                "6,6000000,Landed,100\n",
            ),
        );
        let sessions = find_sessions(&log, 0.0);
        let session = &sessions[0];
        let apogee = session.apogee_row.expect("the altitude column peaks");

        let window = airbrakes_window(session);
        assert_eq!(window.start, session.plot_start);
        assert_eq!(window.end, apogee + 1, "apogee's own row must be drawn");
        assert!(
            window.end < session.flight_end,
            "the descent is not drawn: window ends at {}, flight at {}",
            window.end,
            session.flight_end
        );
    }

    /// A flight that never recorded an apogee — it never left the pad, or the
    /// log stops mid-ascent — is exactly the one someone is opening the figure
    /// to look at, so it gets the whole window rather than an empty one.
    #[test]
    fn an_airbrakes_window_without_an_apogee_falls_back_to_the_whole_flight() {
        let log = log_csv::test_support::log_from_csv(
            "ab_window_no_apogee",
            "record_count,timestamp_us,flight_stage\n0,0,Armed\n1,1000000,Armed\n",
        );
        let sessions = find_sessions(&log, 0.0);
        let session = &sessions[0];
        assert!(session.apogee_row.is_none());

        let window = airbrakes_window(session);
        assert_eq!(window.start, session.plot_start);
        assert_eq!(window.end, session.flight_end);
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
