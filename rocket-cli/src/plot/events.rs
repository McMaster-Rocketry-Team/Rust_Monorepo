//! The moments worth drawing a rule through.
//!
//! An event is anything that happened at an instant rather than over a stretch:
//! a stage transition, a pyro firing, motor burnout, apogee. Marking them with a
//! rule that crosses every panel is what lets one panel be read against another
//! — "the tilt started growing before burnout, not after" is a question about
//! two panels and a vertical line.
//!
//! Two sources are used deliberately rather than one. The `flight_stage`
//! transition says when the flight computer *decided* something; the pyro flag
//! says when the charge actually went. They are usually within a few hundred
//! milliseconds and get merged, but when they are not, that gap is the finding.

use crate::plot::theme::{MACH_LOCKOUT, stage_name};

#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    /// Seconds on the figure's axis.
    pub at_s: f64,
    pub label: String,
}

/// Rising edge indices of a flag column, within `start..end`.
///
/// A column that starts already true counts its first row as an edge: the log
/// beginning mid-event is still the first time this figure can show it.
fn rising_edges(values: &[f32], start: usize, end: usize) -> Vec<usize> {
    let end = end.min(values.len());
    let mut edges = Vec::new();
    let mut prev = false;
    for i in start..end {
        let on = values[i] >= 0.5;
        if on && !prev {
            edges.push(i);
        }
        // `NaN` is neither on nor off. Treating it as off would manufacture a
        // fresh rising edge every time an intermittent column came back.
        if values[i].is_finite() {
            prev = on;
        }
    }
    edges
}

/// Collect every event in the window, in time order, with near-simultaneous
/// ones merged.
///
/// `merge_within` is a duration, not a pixel count — the caller knows the axis
/// span and this stays independent of how wide the figure ends up.
pub fn detect(
    times: &[f64],
    stages: &[Option<u8>],
    burnout: Option<&[f32]>,
    drogue_fire: Option<&[f32]>,
    main_fire: Option<&[f32]>,
    apogee_row: Option<usize>,
    start: usize,
    end: usize,
    merge_within: f64,
) -> Vec<Event> {
    let end = end.min(times.len()).min(stages.len());
    if start >= end {
        return Vec::new();
    }
    let mut raw: Vec<(f64, String)> = Vec::new();

    // Stage transitions, except the one at the window's own start — the flight
    // begins with a transition into Ascent by construction, and a rule on the
    // left edge marks nothing. The first row that carries a stage only seeds
    // the comparison.
    //
    // Compared against the last row that named a stage rather than against the
    // row before, which is not the same thing in a log where a row exists
    // because some record was written and not because every column had a
    // value: a change that happened to land on a blank row used to be dropped
    // entirely, leaving a background band that begins where no rule does.
    let mut previous: Option<u8> = None;
    for i in start..end {
        let Some(stage) = stages[i] else { continue };
        if let Some(prev) = previous
            && prev != stage
        {
            // Ascent-onset is the flight computer deciding the motor has lit,
            // which is what "ignition" names. It is not called liftoff because
            // the detector is an acceleration threshold, not a break-wire: it
            // fires while the rocket is still on the rail, and the burn band
            // this rule opens is measured from it.
            // Leaving the pad is "ignition" whichever airborne stage comes
            // first — on the deployment figure that is the Mach lockout
            // rather than `Ascent`, and both name the same moment: the
            // flight computer decided the motor lit, which is what T+0 is
            // measured from.
            let label = if prev <= 2 && matches!(stage, 3 | MACH_LOCKOUT) {
                "ignition"
            } else {
                stage_name(stage)
            };
            raw.push((times[i], label.to_string()));
        }
        previous = Some(stage);
    }

    for (column, label) in [
        (burnout, "burnout"),
        (drogue_fire, "drogue fire"),
        (main_fire, "main fire"),
    ] {
        if let Some(values) = column {
            for i in rising_edges(values, start, end) {
                raw.push((times[i], label.to_string()));
            }
        }
    }

    if let Some(row) = apogee_row
        && row >= start
        && row < end
    {
        raw.push((times[row], "apogee".to_string()));
    }

    raw.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Merge what a reader could not tell apart anyway. "Drogue" and "drogue
    // fire" land within a sample or two of each other on a healthy flight, and
    // two rules a pixel apart with overlapping labels is worse than one rule
    // naming both.
    let mut merged: Vec<Event> = Vec::new();
    for (at_s, label) in raw {
        match merged.last_mut() {
            Some(last) if at_s - last.at_s <= merge_within => {
                if !last.label.split(" / ").any(|part| part == label) {
                    last.label.push_str(" / ");
                    last.label.push_str(&label);
                }
            }
            _ => merged.push(Event { at_s, label }),
        }
    }
    merged
}

#[cfg(test)]
mod tests {
    use super::*;

    fn times(n: usize) -> Vec<f64> {
        (0..n).map(|i| i as f64).collect()
    }

    #[test]
    fn stage_transitions_become_events_but_the_windows_own_start_does_not() {
        let stages = vec![Some(3), Some(3), Some(4), Some(4), Some(5)];
        let events = detect(&times(5), &stages, None, None, None, None, 0, 5, 0.0);
        assert_eq!(
            events,
            vec![
                Event { at_s: 2.0, label: "Drogue".into() },
                Event { at_s: 4.0, label: "Main".into() },
            ]
        );
    }

    /// The decision and the charge are separate facts. When they coincide they
    /// read as one rule; the merge must not lose either name.
    #[test]
    fn a_stage_change_and_its_pyro_merge_into_one_labelled_rule() {
        let stages = vec![Some(3), Some(3), Some(4), Some(4)];
        let fire = vec![0.0f32, 0.0, 0.0, 1.0];
        let events = detect(&times(4), &stages, None, Some(&fire), None, None, 0, 4, 2.0);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].label, "Drogue / drogue fire");
        assert_eq!(events[0].at_s, 2.0);
    }

    /// ...and when they do NOT coincide, that gap is the whole point, so they
    /// stay as two rules.
    #[test]
    fn a_pyro_that_lags_its_stage_change_stays_a_separate_rule() {
        let stages = vec![Some(3), Some(4), Some(4), Some(4), Some(4)];
        let fire = vec![0.0f32, 0.0, 0.0, 0.0, 1.0];
        let events = detect(&times(5), &stages, None, Some(&fire), None, None, 0, 5, 1.0);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].label, "Drogue");
        assert_eq!(events[1].label, "drogue fire");
    }

    /// A flag that drops out and returns has not fired twice.
    #[test]
    fn an_intermittent_flag_does_not_fire_again_when_it_comes_back() {
        let values = vec![0.0f32, 1.0, 1.0, f32::NAN, f32::NAN, 1.0, 1.0];
        assert_eq!(rising_edges(&values, 0, 7), vec![1]);
    }

    #[test]
    fn a_flag_already_set_at_the_window_start_counts_once() {
        let values = vec![1.0f32, 1.0, 0.0, 1.0];
        assert_eq!(rising_edges(&values, 0, 4), vec![0, 3]);
    }

    #[test]
    fn apogee_outside_the_window_is_not_marked() {
        let stages = vec![Some(3); 4];
        let events = detect(&times(4), &stages, None, None, None, Some(9), 0, 4, 0.0);
        assert!(events.is_empty());
    }

    /// A gap in `flight_stage` must not read as a transition when it resumes on
    /// the same stage it left.
    #[test]
    fn a_blank_stage_cell_does_not_manufacture_a_transition() {
        let stages = vec![Some(3), None, Some(3), Some(4)];
        let events = detect(&times(4), &stages, None, None, None, None, 0, 4, 0.0);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].label, "Drogue");
    }

    /// The other half of that: a change that lands across a blank row is still
    /// a change. Dropping it left the stage band starting at a boundary with
    /// no rule through it.
    #[test]
    fn a_transition_across_a_blank_row_is_still_found() {
        let stages = vec![Some(3), Some(3), None, Some(4), Some(4)];
        let events = detect(&times(5), &stages, None, None, None, None, 0, 5, 0.0);
        assert_eq!(
            events,
            vec![Event { at_s: 3.0, label: "Drogue".into() }]
        );
    }
}
