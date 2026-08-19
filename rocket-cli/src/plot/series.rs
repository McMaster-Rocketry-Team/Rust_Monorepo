//! Turning a raw column into something a 1920-pixel-wide chart can draw.
//!
//! A flight log holds a couple of hundred thousand rows and a panel is under two
//! thousand pixels wide, so something has to give. Plotting every point makes
//! plotters walk ~100 line segments per pixel column, and the result is
//! indistinguishable from the decimated one because they land on the same pixel
//! anyway.
//!
//! The reduction here is min/max decimation: each pixel column keeps the highest
//! and lowest sample that falls in it, emitted in the order they occurred.
//! Naive stride sampling ("every 100th row") would be simpler and is wrong for
//! this data — a pyro pulse, a filter divergence, a single-sample accelerometer
//! spike are all one or two rows wide, and stride sampling deletes exactly those
//! while leaving the smooth parts untouched. Min/max keeps the envelope, so a
//! transient still shows up as a full-height spike.

/// One column reduced to drawable form.
pub struct Trace {
    /// Contiguous stretches of data. A new run starts wherever the column had
    /// no finite sample for a whole pixel column, so absent data draws as a gap
    /// rather than as a straight line bridging it — which would otherwise
    /// invent a reading that was never taken.
    pub runs: Vec<Vec<(f64, f32)>>,
    pub min: f32,
    pub max: f32,
}

/// Reduce `values[start..end]` against the times in `times`.
///
/// Returns `None` when the range holds no finite sample at all, which is how a
/// panel learns to say "not recorded" instead of drawing empty axes.
pub fn decimate(
    times: &[f64],
    values: &[f32],
    start: usize,
    end: usize,
    buckets: usize,
) -> Option<Trace> {
    let end = end.min(values.len()).min(times.len());
    if start >= end || buckets == 0 {
        return None;
    }

    let span = end - start;
    let buckets = buckets.min(span);
    let mut runs: Vec<Vec<(f64, f32)>> = Vec::new();
    let mut current: Vec<(f64, f32)> = Vec::new();
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;

    for b in 0..buckets {
        // Computed from the bucket index rather than accumulated so rounding
        // cannot drift and leave the last bucket short or past the end.
        let lo = start + span * b / buckets;
        let hi = start + span * (b + 1) / buckets;

        let mut lo_pt: Option<(usize, f32)> = None;
        let mut hi_pt: Option<(usize, f32)> = None;
        for i in lo..hi {
            let v = values[i];
            if !v.is_finite() {
                continue;
            }
            if lo_pt.is_none_or(|(_, best)| v < best) {
                lo_pt = Some((i, v));
            }
            if hi_pt.is_none_or(|(_, best)| v > best) {
                hi_pt = Some((i, v));
            }
        }

        match (lo_pt, hi_pt) {
            (Some(a), Some(b_pt)) => {
                min = min.min(a.1);
                max = max.max(b_pt.1);
                // Chronological order, so the drawn line does not zig backwards
                // inside the pixel column.
                let (first, second) = if a.0 <= b_pt.0 { (a, b_pt) } else { (b_pt, a) };
                current.push((times[first.0], first.1));
                if second.0 != first.0 {
                    current.push((times[second.0], second.1));
                }
            }
            // Nothing finite in this whole pixel column: a real gap.
            _ => {
                if !current.is_empty() {
                    runs.push(std::mem::take(&mut current));
                }
            }
        }
    }
    if !current.is_empty() {
        runs.push(current);
    }

    if runs.is_empty() {
        return None;
    }
    Some(Trace { runs, min, max })
}

/// How long a column may say nothing before the silence means something.
///
/// A CSV row exists because *some* record was logged, not because every column
/// had a value: the slow telemetry record carries no estimator state, so a
/// fast flag reads `NaN` on every row the slow record produced — scattered
/// single rows, a few milliseconds of quiet each. That is not the flag going
/// false, and treating it as false shatters one continuous band into one
/// rectangle per interleaved row.
///
/// The opposite mistake is just as real: a node that drops off the bus stops
/// logging its flags entirely, and a band drawn through that would claim a
/// state nobody reported for the rest of the flight.
///
/// So the threshold is taken from the column's own cadence rather than from a
/// constant: ten times the median spacing of the rows that did carry a value.
/// A 1 kHz flag tolerates a couple of dozen milliseconds of quiet, a 1 Hz flag
/// tolerates ten seconds, and neither number has to be known here. Columns too
/// sparse to establish a cadence get no threshold — nothing can be inferred
/// from silence in a column that is almost all silence.
fn silence_threshold(times: &[f64], present: impl Fn(usize) -> bool, start: usize, end: usize) -> f64 {
    let mut gaps: Vec<f64> = Vec::new();
    let mut prev: Option<f64> = None;
    for i in start..end {
        if present(i) {
            if let Some(p) = prev
                && times[i] > p
            {
                gaps.push(times[i] - p);
            }
            prev = Some(times[i]);
        }
    }
    if gaps.is_empty() {
        return f64::INFINITY;
    }
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    gaps[gaps.len() / 2] * 10.0
}

/// Time ranges where a flag column reads true, as `(from_s, to_s)`.
///
/// Computed on raw rows rather than on decimated ones: a pyro fire flag is set
/// for a handful of rows and decimation to pixel columns would be free to lose
/// which side of the boundary it fell on. The spans are merged afterwards
/// instead, which keeps the edges exact.
///
/// `NaN` is not false, and it is not true either: it is a row that did not
/// carry this column, so it neither opens a span nor closes one. What closes a
/// span is silence long enough to mean it — see [`silence_threshold`] — and
/// then it closes at the last row that actually said "true", never at the row
/// where the log resumed.
pub fn true_spans(times: &[f64], values: &[f32], start: usize, end: usize) -> Vec<(f64, f64)> {
    let end = end.min(values.len()).min(times.len());
    let max_gap = silence_threshold(times, |i| values[i].is_finite(), start, end);
    let mut spans = Vec::new();
    let mut open: Option<f64> = None;
    let mut spoke_at: Option<f64> = None;
    for i in start..end {
        if !values[i].is_finite() {
            continue;
        }
        if let (Some(from), Some(prev)) = (open, spoke_at)
            && times[i] - prev > max_gap
        {
            spans.push((from, prev));
            open = None;
        }
        let on = values[i] >= 0.5;
        match (open, on) {
            (None, true) => open = Some(times[i]),
            (Some(from), false) => {
                spans.push((from, times[i]));
                open = None;
            }
            _ => {}
        }
        spoke_at = Some(times[i]);
    }
    if let Some(from) = open
        && let Some(last) = spoke_at
    {
        spans.push((from, last));
    }
    spans
}

/// Widen spans to a pixel floor, clip them to the axis, and merge what then
/// touches.
///
/// The merge is the point. Spans are drawn as translucent rectangles, and two
/// of them over the same pixels wash it twice — so a band that happens to be
/// finely divided comes out darker than one that is not, which reads as a
/// stronger claim about a state that is merely binary. Widening a short pulse
/// to three pixels is what makes the divisions overlap in the first place, so
/// the merge belongs here, after the widening, rather than at either end.
pub fn widen_and_merge(spans: &[(f64, f64)], min_width: f64, clip: (f64, f64)) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for &(a, b) in spans {
        let a = a.max(clip.0);
        let b = b.max(a + min_width).min(clip.1);
        if b <= a {
            continue;
        }
        match out.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// Contiguous runs of one `flight_stage` value, as `(from_s, to_s, stage)`.
///
/// A row with no stage is read the same way [`true_spans`] reads a `NaN`: as a
/// row that did not carry the column, not as the stage ending. Only silence
/// past this column's own cadence closes a band, and it closes at the last row
/// that named a stage.
pub fn stage_spans(
    times: &[f64],
    stages: &[Option<u8>],
    start: usize,
    end: usize,
) -> Vec<(f64, f64, u8)> {
    let end = end.min(stages.len()).min(times.len());
    let max_gap = silence_threshold(times, |i| stages[i].is_some(), start, end);
    let mut spans: Vec<(f64, f64, u8)> = Vec::new();
    let mut open: Option<(f64, u8)> = None;
    let mut spoke_at: Option<f64> = None;
    for i in start..end {
        let Some(stage) = stages[i] else { continue };
        if let (Some((from, prev)), Some(at)) = (open, spoke_at)
            && times[i] - at > max_gap
        {
            spans.push((from, at, prev));
            open = None;
        }
        match open {
            None => open = Some((times[i], stage)),
            Some((from, prev)) if stage != prev => {
                spans.push((from, times[i], prev));
                open = Some((times[i], stage));
            }
            _ => {}
        }
        spoke_at = Some(times[i]);
    }
    if let Some((from, s)) = open
        && let Some(last) = spoke_at
    {
        spans.push((from, last, s));
    }
    spans
}

#[cfg(test)]
mod tests {
    use super::*;

    fn times(n: usize) -> Vec<f64> {
        (0..n).map(|i| i as f64).collect()
    }

    /// The property stride sampling fails. One row out of a thousand carries the
    /// spike; reducing to 10 buckets must still show it at full height.
    #[test]
    fn a_one_row_spike_survives_reduction_to_a_tenth_of_a_percent() {
        let mut values = vec![0.0f32; 1000];
        values[517] = 42.0;
        let trace = decimate(&times(1000), &values, 0, 1000, 10).unwrap();
        assert_eq!(trace.max, 42.0);
        assert!(
            trace
                .runs
                .iter()
                .flatten()
                .any(|&(t, v)| v == 42.0 && t == 517.0)
        );
    }

    /// Reduction must not reorder time — a bucket whose max precedes its min
    /// emits the max first.
    #[test]
    fn points_within_a_bucket_stay_in_chronological_order() {
        let values = vec![5.0f32, 9.0, 1.0, 4.0];
        let trace = decimate(&times(4), &values, 0, 4, 1).unwrap();
        let pts: Vec<f64> = trace.runs[0].iter().map(|&(t, _)| t).collect();
        assert!(pts.windows(2).all(|w| w[0] <= w[1]), "{pts:?}");
        assert_eq!(trace.runs[0].first().unwrap().1, 9.0);
        assert_eq!(trace.runs[0].last().unwrap().1, 1.0);
    }

    /// A stretch with no data breaks the line instead of bridging it. This is
    /// the mag column in the sample log, present for under a quarter of rows.
    #[test]
    fn absent_data_becomes_a_gap_not_a_bridge() {
        let mut values = vec![1.0f32; 100];
        values[40..60].fill(f32::NAN);
        let trace = decimate(&times(100), &values, 0, 100, 10).unwrap();
        assert_eq!(trace.runs.len(), 2, "expected a break across the NaN run");
    }

    /// A low-rate column sampled into every bucket stays one continuous run —
    /// gaps are for buckets with nothing at all, not for sparse ones.
    #[test]
    fn a_sparse_but_regular_column_does_not_fragment() {
        let values: Vec<f32> = (0..100)
            .map(|i| if i % 5 == 0 { i as f32 } else { f32::NAN })
            .collect();
        let trace = decimate(&times(100), &values, 0, 100, 10).unwrap();
        assert_eq!(trace.runs.len(), 1);
        // Two samples per bucket survive as that bucket's min and max.
        assert_eq!(trace.runs[0].len(), 20);
    }

    #[test]
    fn a_column_with_nothing_finite_reduces_to_nothing() {
        assert!(decimate(&times(50), &vec![f32::NAN; 50], 0, 50, 10).is_none());
    }

    #[test]
    fn flag_spans_capture_each_pulse_and_close_the_last_one() {
        let values = vec![0.0f32, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0];
        let spans = true_spans(&times(7), &values, 0, 7);
        assert_eq!(spans, vec![(1.0, 3.0), (5.0, 6.0)]);
    }

    /// The interleaved-record case, which is what every real log looks like:
    /// the slow record carries no estimator state, so the fast flag is blank on
    /// scattered single rows. Reading those as false shattered one band into
    /// one rectangle per blank row, and the translucent fills then stacked into
    /// a gradient across what is a single binary state.
    #[test]
    fn a_row_that_did_not_carry_the_flag_does_not_break_the_span() {
        let mut values = vec![1.0f32; 40];
        for i in (3..40).step_by(7) {
            values[i] = f32::NAN;
        }
        assert_eq!(true_spans(&times(40), &values, 0, 40), vec![(0.0, 39.0)]);
    }

    /// The other half of the rule: silence far past the column's own cadence is
    /// the log going quiet — a node off the bus — and the band ends at the last
    /// row that reported, not where the log happens to resume.
    #[test]
    fn silence_past_the_columns_cadence_does_close_the_span() {
        let mut values = vec![1.0f32; 60];
        for v in values.iter_mut().take(50).skip(10) {
            *v = f32::NAN;
        }
        assert_eq!(
            true_spans(&times(60), &values, 0, 60),
            vec![(0.0, 9.0), (50.0, 59.0)]
        );
    }

    /// Same reading for the stage column, which fragments the background bands
    /// the same way when a row does not carry it.
    #[test]
    fn stage_bands_survive_rows_that_did_not_carry_a_stage() {
        let mut stages: Vec<Option<u8>> = vec![Some(3); 30];
        for i in (2..30).step_by(5) {
            stages[i] = None;
        }
        assert_eq!(stage_spans(&times(30), &stages, 0, 30), vec![(0.0, 29.0, 3)]);
    }

    /// Widening a sub-pixel pulse is what makes neighbours overlap, so the
    /// merge has to happen after it, and the clip has to survive both.
    #[test]
    fn widening_merges_what_it_pushes_together() {
        let spans = [(1.0, 1.01), (1.2, 1.21), (5.0, 6.0)];
        assert_eq!(
            widen_and_merge(&spans, 0.5, (0.0, 10.0)),
            vec![(1.0, 1.7), (5.0, 6.0)]
        );
    }

    #[test]
    fn stage_spans_break_at_every_transition() {
        let stages = vec![Some(2), Some(2), Some(3), Some(4), Some(4)];
        assert_eq!(
            stage_spans(&times(5), &stages, 0, 5),
            vec![(0.0, 2.0, 2), (2.0, 3.0, 3), (3.0, 4.0, 4)]
        );
    }
}
