//! Finding the flights inside one CSV.
//!
//! Two separate cuts happen here, and they answer different questions.
//!
//! The first is the **session** split. The VLF5 logger resumes the existing log
//! across power cycles instead of starting a new one, so a single card — and so
//! a single downloaded CSV — holds every armed session since the last
//! `clear-flight-log`, concatenated with nothing between them. `record_count`
//! restarting at zero is the only mark of a boundary, exactly as it is for
//! `merge_log_records` on the download side.
//!
//! The second is the **flight window** inside a session. Logging runs for as
//! long as armed mode does, which starts on the pad and ends after landing, so a
//! session brackets the flight with idle time at both ends — twenty seconds of
//! pad in the sample log, and over two minutes of sitting in a field in a real
//! recovery. Plotting that compresses the interesting part into a sliver.

use crate::plot::log_csv::FlightLog;

/// Stage discriminants, named so the window logic reads as intent rather than
/// as magic numbers. These mirror `FlightStage` in
/// `firmware_common_new::can_bus::messages::vl_status`.
const STAGE_ASCENT: u8 = 3;
const STAGE_LANDED: u8 = 6;
const STAGE_FAILED_TO_REACH_MIN_APOGEE: u8 = 7;

/// True for any stage that means "off the pad and not yet done".
///
/// `FailedToReachMinApogee` counts. It is a flight that went wrong, which is
/// precisely the flight someone is opening this plot to look at.
fn is_airborne(stage: u8) -> bool {
    matches!(
        stage,
        STAGE_ASCENT | 4 /* DrogueChute */ | 5 /* MainChute */ | STAGE_FAILED_TO_REACH_MIN_APOGEE
    )
}

/// How the flight window was arrived at, so the caller can say so rather than
/// presenting a guess with the same confidence as a reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WindowSource {
    /// Both ends came from `flight_stage` transitions.
    Stages,
    /// Liftoff came from a stage transition; the log ends before `Landed`.
    /// Normal for a log pulled from a rocket that was still descending, and for
    /// every HIL run.
    StagesNoLanding,
    /// No airborne stage anywhere in the session. The whole session is used.
    NeverLeftThePad,
}

/// One armed session, and the flight inside it.
#[derive(Debug, Clone)]
pub struct Session {
    /// Row range of the whole session, `start..end`.
    pub start: usize,
    pub end: usize,
    /// Row range of the flight itself, always within `start..end`.
    pub flight_start: usize,
    pub flight_end: usize,
    pub window_source: WindowSource,
    /// Wall-clock start, if the session ever had a GPS-disciplined clock.
    pub unix_time_us: Option<f64>,
    /// Peak of whichever altitude estimator reported, and the row it was on.
    ///
    /// A row index rather than a time, because "when" only has meaning once an
    /// origin is chosen and this type does not get to choose it — the figures
    /// measure from liftoff, which is not where the file starts.
    pub apogee_asl: Option<f32>,
    pub apogee_row: Option<usize>,
}

impl Session {
    pub fn flight_rows(&self) -> usize {
        self.flight_end - self.flight_start
    }

    /// Flight duration in seconds.
    pub fn duration_s(&self, log: &FlightLog) -> f64 {
        if self.flight_rows() < 2 {
            return 0.0;
        }
        (log.timestamp_us[self.flight_end - 1] - log.timestamp_us[self.flight_start]) / 1e6
    }

    /// Seconds from the session's first row to the flight's first row — the
    /// pad idle that got trimmed off the front.
    pub fn trimmed_before_s(&self, log: &FlightLog) -> f64 {
        (log.timestamp_us[self.flight_start] - log.timestamp_us[self.start]) / 1e6
    }

    /// Seconds from the flight's last row to the session's last row.
    pub fn trimmed_after_s(&self, log: &FlightLog) -> f64 {
        (log.timestamp_us[self.end - 1] - log.timestamp_us[self.flight_end - 1]) / 1e6
    }
}

/// Split into sessions, then locate the flight in each.
pub fn find_sessions(log: &FlightLog) -> Vec<Session> {
    let mut bounds = vec![0usize];
    for i in 1..log.row_count {
        // The same test `merge_log_records` uses. `sequence` is reset to 0 on
        // entry to `log_flight_data`, so it only ever decreases across a
        // boundary; within a session it climbs, skipping numbers where a record
        // was dropped but never going backwards.
        if log.record_count[i] < log.record_count[i - 1] {
            bounds.push(i);
        }
    }
    bounds.push(log.row_count);

    bounds
        .windows(2)
        .map(|w| build_session(log, w[0], w[1]))
        .collect()
}

fn build_session(log: &FlightLog, start: usize, end: usize) -> Session {
    let first_airborne = (start..end).find(|&i| log.stage[i].is_some_and(is_airborne));

    let (flight_start, flight_end, window_source) = match first_airborne {
        Some(liftoff) => {
            // The first `Landed` at or after liftoff ends it. Searching forward
            // from liftoff rather than from the session start matters: a session
            // that begins with a stale `Landed` carried over from the previous
            // flight would otherwise close the window before it opened.
            let landed = (liftoff..end).find(|&i| log.stage[i] == Some(STAGE_LANDED));
            match landed {
                // `landed` is the first row of the ground, so it is the
                // exclusive end — the flight is everything before it.
                Some(landed) => (liftoff, landed.max(liftoff + 1), WindowSource::Stages),
                None => (liftoff, end, WindowSource::StagesNoLanding),
            }
        }
        // Armed, then disarmed without launching — a pad abort or a bench run.
        // There is no flight to trim to, so show the session as it is rather
        // than showing nothing.
        None => (start, end, WindowSource::NeverLeftThePad),
    };

    let (apogee_asl, apogee_row) = peak_altitude(log, flight_start, flight_end);

    Session {
        start,
        end,
        flight_start,
        flight_end,
        window_source,
        unix_time_us: first_finite(log.column("unix_time_us"), start, end),
        apogee_asl,
        apogee_row,
    }
}

/// Highest altitude either estimator reported inside the window.
///
/// Both are consulted because they fail independently: the deployment filter is
/// frozen through the Mach lockout and the airbrakes filter only runs while
/// airbrakes are enabled, so on any given flight one of them may be absent for
/// exactly the stretch that contains apogee.
fn peak_altitude(log: &FlightLog, start: usize, end: usize) -> (Option<f32>, Option<usize>) {
    let mut best: Option<(f32, usize)> = None;
    for name in [
        "deployment_kf_altitude_asl",
        "airbrakes_kf_altitude_asl",
        "gps_altitude_asl",
    ] {
        let Some(values) = log.column(name) else {
            continue;
        };
        for i in start..end.min(values.len()) {
            let v = values[i];
            if v.is_finite() && best.is_none_or(|(b, _)| v > b) {
                best = Some((v, i));
            }
        }
    }
    match best {
        Some((v, i)) => (Some(v), Some(i)),
        None => (None, None),
    }
}

fn first_finite(column: Option<&[f32]>, start: usize, end: usize) -> Option<f64> {
    let column = column?;
    (start..end.min(column.len()))
        .map(|i| column[i])
        .find(|v| v.is_finite() && *v > 0.0)
        .map(|v| v as f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plot::log_csv::test_support::log_from_csv;

    /// Build a log from `(record_count, stage)` pairs at a fixed 100 ms tick.
    /// `name` keeps parallel test threads off each other's temp files.
    fn log_from(name: &str, rows: &[(u32, &str)]) -> FlightLog {
        let mut body = String::from("record_count,timestamp_us,flight_stage\n");
        for (i, (count, stage)) in rows.iter().enumerate() {
            body.push_str(&format!("{},{},{}\n", count, i * 100_000, stage));
        }
        log_from_csv(&format!("session_{name}"), &body)
    }

    /// The pad idle at the front and the field time at the back are exactly what
    /// the window exists to remove.
    #[test]
    fn the_window_is_liftoff_to_landing_not_the_whole_session() {
        let log = log_from("window", &[
            (0, "Armed"),
            (1, "Armed"),
            (2, "Ascent"),
            (3, "DrogueChute"),
            (4, "MainChute"),
            (5, "Landed"),
            (6, "Landed"),
        ]);
        let sessions = find_sessions(&log);
        assert_eq!(sessions.len(), 1);
        let s = &sessions[0];
        assert_eq!((s.start, s.end), (0, 7));
        assert_eq!((s.flight_start, s.flight_end), (2, 5));
        assert_eq!(s.window_source, WindowSource::Stages);
        assert_eq!(s.trimmed_before_s(&log), 0.2);
        assert_eq!(s.trimmed_after_s(&log), 0.2);
    }

    /// `record_count` restarting is the only boundary marker there is, and each
    /// session gets its own window rather than one window spanning both.
    #[test]
    fn a_resumed_log_splits_into_one_session_per_arming() {
        let log = log_from("resumed", &[
            (0, "Armed"),
            (1, "Ascent"),
            (2, "Landed"),
            // Power cycle: `sequence` restarts.
            (0, "Armed"),
            (1, "Ascent"),
            (2, "DrogueChute"),
            (3, "Landed"),
        ]);
        let sessions = find_sessions(&log);
        assert_eq!(sessions.len(), 2);
        assert_eq!((sessions[0].flight_start, sessions[0].flight_end), (1, 2));
        assert_eq!((sessions[1].flight_start, sessions[1].flight_end), (4, 6));
    }

    /// Every HIL run and any log pulled from a rocket still under canopy ends
    /// mid-descent. That is not a parse failure and must not truncate to
    /// nothing — the sample log this feature was built against is exactly this
    /// shape.
    #[test]
    fn a_log_that_ends_before_landing_runs_to_the_end_of_the_session() {
        let log = log_from("no_landing", &[
            (0, "Armed"),
            (1, "Ascent"),
            (2, "DrogueChute"),
            (3, "MainChute"),
        ]);
        let s = &find_sessions(&log)[0];
        assert_eq!((s.flight_start, s.flight_end), (1, 4));
        assert_eq!(s.window_source, WindowSource::StagesNoLanding);
    }

    /// Armed, never launched. Trimming to a flight that does not exist would
    /// leave an empty chart; showing the session is the useful answer.
    #[test]
    fn a_session_that_never_launched_keeps_all_of_its_rows() {
        let log = log_from("never_launched", &[(0, "Armed"), (1, "Armed"), (2, "Armed")]);
        let s = &find_sessions(&log)[0];
        assert_eq!((s.flight_start, s.flight_end), (0, 3));
        assert_eq!(s.window_source, WindowSource::NeverLeftThePad);
    }

    /// A session opening on a stale `Landed` must not close the window at row
    /// zero and report a flight that ended before it began.
    #[test]
    fn a_landed_row_before_liftoff_does_not_close_the_window() {
        let log = log_from("stale_landed", &[
            (0, "Landed"),
            (1, "Armed"),
            (2, "Ascent"),
            (3, "DrogueChute"),
            (4, "Landed"),
        ]);
        let s = &find_sessions(&log)[0];
        assert_eq!((s.flight_start, s.flight_end), (2, 4));
    }

    /// A flight that never reached minimum apogee is still a flight.
    #[test]
    fn a_failed_apogee_flight_is_treated_as_airborne() {
        let log = log_from("failed_apogee", &[
            (0, "Armed"),
            (1, "FailedToReachMinApogee"),
            (2, "Landed"),
        ]);
        let s = &find_sessions(&log)[0];
        assert_eq!((s.flight_start, s.flight_end), (1, 2));
    }
}
