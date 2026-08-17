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

/// Time ranges where a flag column reads true, as `(from_s, to_s)`.
///
/// Computed on raw rows rather than on decimated ones: a pyro fire flag is set
/// for a handful of rows and decimation to pixel columns would be free to lose
/// which side of the boundary it fell on. The spans are merged afterwards
/// instead, which keeps the edges exact.
///
/// `NaN` ends a span. An absent flag is not a false one, and drawing it as false
/// would assert continuity across a stretch where the log says nothing.
pub fn true_spans(times: &[f64], values: &[f32], start: usize, end: usize) -> Vec<(f64, f64)> {
    let end = end.min(values.len()).min(times.len());
    let mut spans = Vec::new();
    let mut open: Option<f64> = None;
    for i in start..end {
        let on = values[i] >= 0.5;
        match (open, on) {
            (None, true) => open = Some(times[i]),
            (Some(from), false) => {
                spans.push((from, times[i]));
                open = None;
            }
            _ => {}
        }
    }
    if let Some(from) = open
        && end > start
    {
        spans.push((from, times[end - 1]));
    }
    spans
}

/// Contiguous runs of one `flight_stage` value, as `(from_s, to_s, stage)`.
pub fn stage_spans(
    times: &[f64],
    stages: &[Option<u8>],
    start: usize,
    end: usize,
) -> Vec<(f64, f64, u8)> {
    let end = end.min(stages.len()).min(times.len());
    let mut spans: Vec<(f64, f64, u8)> = Vec::new();
    let mut open: Option<(f64, u8)> = None;
    for i in start..end {
        match (open, stages[i]) {
            (None, Some(s)) => open = Some((times[i], s)),
            (Some((from, prev)), Some(s)) if s != prev => {
                spans.push((from, times[i], prev));
                open = Some((times[i], s));
            }
            (Some((from, prev)), None) => {
                spans.push((from, times[i], prev));
                open = None;
            }
            _ => {}
        }
    }
    if let Some((from, s)) = open
        && end > start
    {
        spans.push((from, times[end - 1], s));
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

    /// An absent flag is not a false one; it ends the span rather than extending
    /// it through a stretch the log says nothing about.
    #[test]
    fn a_nan_flag_closes_the_span_rather_than_reading_as_true() {
        let values = vec![1.0f32, 1.0, f32::NAN, f32::NAN, 1.0];
        assert_eq!(
            true_spans(&times(5), &values, 0, 5),
            vec![(0.0, 2.0), (4.0, 4.0)]
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
