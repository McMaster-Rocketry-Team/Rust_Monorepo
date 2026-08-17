//! Rendering the two 1920×1080 figures.
//!
//! Both share one time axis that starts at T+0 = liftoff and ends at landing,
//! so a point read off one figure can be found at the same x on the other.
//! Every panel gets the flight-stage bands washed in behind it for the same
//! reason: it means no panel has to be read in isolation to know whether a
//! feature happened under thrust, under drogue, or under main.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Result, anyhow};
use plotters::coord::Shift;
use plotters::coord::types::RangedCoordf64;
use plotters::prelude::*;
use plotters::style::text_anchor::{HPos, Pos, VPos};

use crate::plot::log_csv::FlightLog;
use crate::plot::series::{Trace, decimate, stage_spans, true_spans};
use crate::plot::session::{Session, WindowSource};
use crate::plot::theme::{self, stage_color};

pub const WIDTH: u32 = 1920;
pub const HEIGHT: u32 = 1080;
/// Height reserved for the figure's own title block.
const HEADER_H: u32 = 68;
/// Width of the y-axis label gutter. One value for every panel on both figures:
/// panels share a time axis, and an axis that starts at a different x on each
/// row cannot be read across rows. It also has to be wide enough for the
/// longest lane name, which is why it is larger than the numbers alone need.
const Y_GUTTER: i32 = 92;

type Area<'a> = DrawingArea<BitMapBackend<'a>, Shift>;

/// plotters' error type is parameterised by the backend and is awkward to carry
/// through `?` into `anyhow`. Every drawing failure here means the same thing —
/// the image could not be produced — so they collapse to one message.
trait PlotErr<T> {
    fn plot(self) -> Result<T>;
}

impl<T, E: std::fmt::Display> PlotErr<T> for std::result::Result<T, E> {
    fn plot(self) -> Result<T> {
        self.map_err(|e| anyhow!("rendering the figure failed: {e}"))
    }
}

/// One trace on a panel.
struct Line {
    label: &'static str,
    column: &'static str,
    color: RGBColor,
    /// Applied after reduction, to bring a column onto the panel's unit — the
    /// payload battery is logged in mV and shares a panel with two voltages.
    scale: f32,
}

impl Line {
    fn new(label: &'static str, column: &'static str, color: RGBColor) -> Self {
        Self {
            label,
            column,
            color,
            scale: 1.0,
        }
    }

    fn scaled(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }
}

struct Panel {
    title: &'static str,
    unit: &'static str,
    lines: Vec<Line>,
    /// Draw a reference line at zero. Only meaningful where the sign carries
    /// information — velocity crossing zero *is* apogee.
    zero_line: bool,
    mark_apogee: bool,
}

impl Panel {
    fn new(title: &'static str, unit: &'static str, lines: Vec<Line>) -> Self {
        Self {
            title,
            unit,
            lines,
            zero_line: false,
            mark_apogee: false,
        }
    }

    fn with_zero(mut self) -> Self {
        self.zero_line = true;
        self
    }

    fn with_apogee(mut self) -> Self {
        self.mark_apogee = true;
        self
    }
}

/// A panel's chosen vertical range.
struct AxisRange {
    lo: f32,
    hi: f32,
    /// Set when the range excludes outliers, holding the true `(min, max)` so
    /// the panel can name what it left off the scale.
    clipped: Option<(f32, f32)>,
}

pub struct Renderer<'a> {
    log: &'a FlightLog,
    session: &'a Session,
    /// Seconds relative to liftoff, per row of the whole log.
    times: Vec<f64>,
    /// Columns computed rather than logged.
    derived: HashMap<&'static str, Vec<f32>>,
    x_range: (f64, f64),
    source_name: String,
}

impl<'a> Renderer<'a> {
    pub fn new(log: &'a FlightLog, session: &'a Session, source_name: String) -> Self {
        let t0 = log.timestamp_us[session.flight_start];
        let times: Vec<f64> = log.timestamp_us.iter().map(|t| (t - t0) / 1e6).collect();

        // Total acceleration is the signal that shows motor burn, and none of
        // the three logged axes shows it alone once the airframe is rotating.
        let mut derived = HashMap::new();
        if let (Some(x), Some(y), Some(z)) = (
            log.column("acc_x"),
            log.column("acc_y"),
            log.column("acc_z"),
        ) {
            let magnitude = (0..log.row_count)
                .map(|i| (x[i] * x[i] + y[i] * y[i] + z[i] * z[i]).sqrt())
                .collect();
            derived.insert("acc_magnitude", magnitude);
        }

        let x_range = (
            times[session.flight_start],
            times[session.flight_end - 1].max(times[session.flight_start] + 0.001),
        );

        Self {
            log,
            session,
            times,
            derived,
            x_range,
            source_name,
        }
    }

    /// Apogee on this figure's axis — seconds after liftoff.
    fn apogee_at_s(&self) -> Option<f64> {
        self.session.apogee_row.map(|row| self.times[row])
    }

    fn column(&self, name: &str) -> Option<&[f32]> {
        self.derived
            .get(name)
            .map(Vec::as_slice)
            .or_else(|| self.log.column(name))
    }

    fn trace(&self, line: &Line, buckets: usize) -> Option<Trace> {
        let values = self.column(line.column)?;
        let mut trace = decimate(
            &self.times,
            values,
            self.session.flight_start,
            self.session.flight_end,
            buckets,
        )?;
        if line.scale != 1.0 {
            for run in &mut trace.runs {
                for point in run.iter_mut() {
                    point.1 *= line.scale;
                }
            }
            trace.min *= line.scale;
            trace.max *= line.scale;
        }
        Some(trace)
    }

    // ---------------------------------------------------------------- figures

    /// The flight itself: where it went, how fast, how hard, and what the
    /// recovery hardware did about it.
    pub fn render_flight(&self, path: &Path) -> Result<()> {
        let root = BitMapBackend::new(path, (WIDTH, HEIGHT)).into_drawing_area();
        root.fill(&theme::BG).plot()?;
        let (header, body) = root.split_vertically(HEADER_H);
        self.draw_header(&header, "Flight")?;

        // Altitude gets the most height because it is the panel everything else
        // is read against; the lane strip gets the least because a lane is
        // legible at any height that fits its label.
        let (p1, rest) = body.split_vertically(232);
        let (p2, rest) = rest.split_vertically(200);
        let (p3, rest) = rest.split_vertically(200);
        let (p4, p5) = rest.split_vertically(190);

        self.draw_panel(
            &p1,
            &Panel::new(
                "Altitude ASL",
                "m",
                vec![
                    Line::new("deployment KF", "deployment_kf_altitude_asl", theme::CYAN),
                    Line::new("airbrakes KF", "airbrakes_kf_altitude_asl", theme::AMBER),
                    Line::new("GPS", "gps_altitude_asl", theme::VIOLET),
                ],
            )
            .with_apogee(),
        )?;
        self.draw_panel(
            &p2,
            &Panel::new(
                "Vertical velocity",
                "m/s",
                vec![
                    Line::new("deployment KF", "deployment_kf_vertical_velocity", theme::CYAN),
                    Line::new("airbrakes KF", "airbrakes_kf_vertical_velocity", theme::AMBER),
                ],
            )
            .with_zero()
            .with_apogee(),
        )?;
        self.draw_panel(
            &p3,
            &Panel::new(
                "Acceleration",
                "m/s²",
                vec![
                    Line::new("x", "acc_x", theme::CORAL),
                    Line::new("y", "acc_y", theme::BLUE),
                    Line::new("z", "acc_z", theme::ROSE),
                    // Last, so it is drawn over the component it coincides
                    // with — on a single-axis airframe that is always one of
                    // them, and the magnitude is the trace being looked for.
                    Line::new("|a|", "acc_magnitude", theme::GREEN),
                ],
            )
            .with_zero(),
        )?;
        self.draw_panel(
            &p4,
            &Panel::new(
                "Air brakes extension",
                "%",
                vec![
                    Line::new("commanded", "air_brakes_commanded_extension", theme::AMBER)
                        .scaled(100.0),
                    Line::new("actual", "air_brakes_actual_extension", theme::CYAN)
                        .scaled(100.0),
                ],
            ),
        )?;
        self.draw_lanes(
            &p5,
            "Pyro & deployment",
            &[
                ("drogue cont.", "pyro_drogue_continuity", theme::CYAN),
                ("drogue fire", "pyro_drogue_fire", theme::AMBER),
                ("main cont.", "pyro_main_continuity", theme::CYAN),
                ("main fire", "pyro_main_fire", theme::AMBER),
                ("short circuit", "pyro_short_circuit", theme::ALERT),
                ("valid. deploy", "air_brakes_validation_deploy", theme::VIOLET),
            ],
        )?;

        root.present().plot()?;
        Ok(())
    }

    /// Everything else that was logged: attitude, environment, power, GPS
    /// quality, the payload rails, and the estimator's own flags.
    pub fn render_misc(&self, path: &Path) -> Result<()> {
        let root = BitMapBackend::new(path, (WIDTH, HEIGHT)).into_drawing_area();
        root.fill(&theme::BG).plot()?;
        let (header, body) = root.split_vertically(HEADER_H);
        self.draw_header(&header, "Auxiliary")?;

        let cells = body.split_evenly((4, 3));
        let panels = [
            Panel::new(
                "Angular rate",
                "°/s",
                vec![
                    Line::new("x", "gyro_x", theme::CORAL),
                    Line::new("y", "gyro_y", theme::BLUE),
                    Line::new("z", "gyro_z", theme::ROSE),
                ],
            )
            .with_zero(),
            Panel::new(
                "Magnetometer",
                "gauss",
                vec![
                    Line::new("x", "mag_x", theme::CORAL),
                    Line::new("y", "mag_y", theme::BLUE),
                    Line::new("z", "mag_z", theme::ROSE),
                ],
            )
            .with_zero(),
            Panel::new(
                "Tilt from vertical",
                "°",
                vec![Line::new("airbrakes KF", "airbrakes_kf_tilt_deg", theme::VIOLET)],
            ),
            Panel::new(
                "Barometric pressure",
                "Pa",
                vec![Line::new("baro", "pressure", theme::CYAN)],
            ),
            Panel::new(
                "Temperature",
                "°C",
                vec![
                    Line::new("avionics", "temperature", theme::AMBER),
                    Line::new("airbrakes servo", "air_brakes_servo_temp", theme::CORAL),
                ],
            ),
            Panel::new(
                "Supply voltage",
                "V",
                vec![
                    Line::new("VL battery", "battery_voltage", theme::GREEN),
                    Line::new("AMP shared", "amp_shared_battery_v", theme::AMBER),
                    // Logged in mV, shown alongside two volt readings.
                    Line::new("payload EPM", "payload_epm_batt_mv", theme::VIOLET).scaled(0.001),
                ],
            ),
            Panel::new(
                "GPS fix quality",
                "count / DOP",
                vec![
                    Line::new("satellites", "num_sats", theme::GREEN),
                    Line::new("HDOP", "hdop", theme::CYAN),
                    Line::new("VDOP", "vdop", theme::AMBER),
                    Line::new("PDOP", "pdop", theme::ROSE),
                ],
            ),
            Panel::new(
                "Apogee prediction",
                "m AGL",
                vec![
                    Line::new("MPC predicted", "mpc_predicted_apogee_agl", theme::CYAN),
                    Line::new("target", "air_brakes_target_apogee_agl", theme::AMBER),
                ],
            ),
            Panel::new(
                "Payload rail current",
                "mA",
                vec![
                    Line::new("sys 3V3", "payload_sys_3v3_ma", theme::CYAN),
                    Line::new("sys 5V", "payload_sys_5v_ma", theme::AMBER),
                    Line::new("per 3V3", "payload_per_3v3_ma", theme::GREEN),
                    Line::new("per 5V", "payload_per_5v_ma", theme::VIOLET),
                    Line::new("per 9V", "payload_per_9v_ma", theme::CORAL),
                    Line::new("per 12V", "payload_per_12v_ma", theme::ROSE),
                ],
            ),
            Panel::new(
                "Payload actuators",
                "steps",
                vec![
                    Line::new("actuator 1", "payload_actuator_1_steps", theme::CYAN),
                    Line::new("actuator 2", "payload_actuator_2_steps", theme::AMBER),
                    Line::new("actuator 3", "payload_actuator_3_steps", theme::GREEN),
                ],
            ),
            Panel::new(
                "CAN node uptime",
                "s",
                vec![
                    Line::new("AMP", "amp_uptime_s", theme::CYAN),
                    Line::new("ICARUS", "icarus_uptime_s", theme::AMBER),
                    Line::new("OZYS", "ozys_uptime_s", theme::GREEN),
                    Line::new("payload SDRM", "payload_sdrm_uptime_s", theme::VIOLET),
                ],
            ),
        ];

        for (cell, panel) in cells.iter().zip(panels.iter()) {
            self.draw_panel(cell, panel)?;
        }
        // The twelfth cell is the flag strip: these are the estimator's own
        // account of why it did what it did, and they only make sense against
        // the same time axis as everything else.
        self.draw_lanes(
            &cells[11],
            "Estimator & baro flags",
            &[
                ("pad calib.", "airbrakes_pad_calibrated", theme::GREEN),
                ("burnout", "airbrakes_burnout", theme::AMBER),
                ("baro trusted", "airbrakes_baro_trusted", theme::CYAN),
                ("subsonic drag", "airbrakes_subsonic_drag", theme::VIOLET),
                ("dep gate rej", "deployment_baro_gate_reject", theme::ALERT),
                ("dep resync", "deployment_baro_resync", theme::ROSE),
                ("ab gate rej", "airbrakes_baro_gate_reject", theme::ALERT),
                ("ab resync", "airbrakes_baro_resync", theme::ROSE),
            ],
        )?;

        root.present().plot()?;
        Ok(())
    }

    // ---------------------------------------------------------------- pieces

    fn draw_header(&self, area: &Area, kind: &str) -> Result<()> {
        area.fill(&theme::BG).plot()?;
        let title = TextStyle::from((theme::FONT, 26).into_font())
            .color(&theme::TEXT)
            .pos(Pos::new(HPos::Left, VPos::Center));
        let sub = TextStyle::from((theme::FONT, 14).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Left, VPos::Center));
        let right = TextStyle::from((theme::FONT, 14).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Right, VPos::Center));

        area.draw_text(&format!("{kind} · {}", self.source_name), &title, (28, 26))
            .plot()?;

        let s = self.session;
        let apogee = match (s.apogee_asl, self.apogee_at_s()) {
            (Some(a), Some(t)) => format!("apogee {a:.0} m ASL at T+{t:.1} s"),
            _ => "apogee not recorded".to_string(),
        };
        area.draw_text(
            &format!(
                "T+0 = liftoff · {:.1} s of flight · {} rows · {}",
                s.duration_s(self.log),
                s.flight_rows(),
                apogee
            ),
            &sub,
            (28, 50),
        )
        .plot()?;

        // The trim is stated rather than assumed. A reader who wonders where the
        // pad time went should not have to guess that it was removed.
        let trim = match s.window_source {
            WindowSource::Stages => format!(
                "trimmed {:.1} s on the pad and {:.1} s after landing",
                s.trimmed_before_s(self.log),
                s.trimmed_after_s(self.log)
            ),
            WindowSource::StagesNoLanding => format!(
                "trimmed {:.1} s on the pad · log ends before landing",
                s.trimmed_before_s(self.log)
            ),
            WindowSource::NeverLeftThePad => {
                "never left the pad · showing the whole session".to_string()
            }
        };
        area.draw_text(&trim, &right, (WIDTH as i32 - 28, 26))
            .plot()?;
        self.draw_stage_key(area)?;
        if let Some(failed) = self.log.crc_failed_rows.filter(|n| *n > 0) {
            let warn = TextStyle::from((theme::FONT, 14).into_font())
                .color(&theme::ALERT)
                .pos(Pos::new(HPos::Right, VPos::Center));
            area.draw_text(
                &format!("{failed} row(s) from CRC-failed blocks"),
                &warn,
                (WIDTH as i32 - 28, 50),
            )
            .plot()?;
        }
        Ok(())
    }

    /// Name the flight-stage bands, in the order they occurred.
    ///
    /// Every panel is washed with these, and an unlabelled wash is just a stain.
    /// The chip is drawn as panel background plus the same translucent fill the
    /// bands use, over an opaque rule in the stage's hue — so the swatch is
    /// literally what the reader sees behind the traces, with enough edge to
    /// find it.
    fn draw_stage_key(&self, area: &Area) -> Result<()> {
        let mut stages: Vec<u8> = Vec::new();
        for (_, _, stage) in self.stage_spans_in_window() {
            if !stages.contains(&stage) {
                stages.push(stage);
            }
        }
        if stages.is_empty() {
            return Ok(());
        }

        const CHIP_W: i32 = 26;
        const CHIP_H: i32 = 13;
        const GAP: i32 = 10;
        let label = |s: u8| theme::stage_name(s);
        // ~6.2 px per character at 12 px sans is close enough to centre the row;
        // being a few pixels off is invisible, and measuring text would mean
        // rendering it twice.
        let widths: Vec<i32> = stages
            .iter()
            .map(|s| CHIP_W + 5 + (label(*s).len() as i32 * 62) / 10 + GAP)
            .collect();
        let total: i32 = widths.iter().sum::<i32>() - GAP;

        let mut x = (WIDTH as i32 - total) / 2;
        let y = 38;
        let text = TextStyle::from((theme::FONT, 12).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Left, VPos::Center));

        for (stage, width) in stages.iter().zip(widths) {
            let chip = [(x, y - CHIP_H / 2), (x + CHIP_W, y + CHIP_H / 2)];
            area.draw(&Rectangle::new(chip, theme::PANEL_BG.filled()))
                .plot()?;
            area.draw(&Rectangle::new(chip, stage_color(*stage).filled()))
                .plot()?;
            area.draw(&Rectangle::new(
                [(x, y + CHIP_H / 2), (x + CHIP_W, y + CHIP_H / 2 + 2)],
                theme::stage_hue(*stage).filled(),
            ))
            .plot()?;
            area.draw_text(label(*stage), &text, (x + CHIP_W + 5, y))
                .plot()?;
            x += width;
        }
        Ok(())
    }

    fn stage_spans_in_window(&self) -> Vec<(f64, f64, u8)> {
        stage_spans(
            &self.times,
            &self.log.stage,
            self.session.flight_start,
            self.session.flight_end,
        )
    }

    /// Pick the vertical range for a panel.
    ///
    /// Usually this is just "everything, plus a margin". The exception is a
    /// panel where a handful of samples are orders of magnitude outside the
    /// rest — a Kalman filter that diverged for three rows at liftoff, say. In
    /// the sample log eight samples out of 192 000 reach 2749 m/s while the
    /// 99.9th percentile is 136, and scaling to the peak squashes the entire
    /// flight into one pixel row.
    ///
    /// So the axis falls back to a percentile range, but the outliers are still
    /// *drawn* — plotters clips them to the panel, which reads as a trace
    /// running off the top, and [`AxisRange::clipped`] makes the figure say so
    /// in words. Nothing is silently discarded; only the scale changes.
    fn axis_range(&self, traces: &[(&Line, Option<Trace>)], include_zero: bool) -> AxisRange {
        let mut values: Vec<f32> = Vec::new();
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        for trace in traces.iter().filter_map(|(_, t)| t.as_ref()) {
            lo = lo.min(trace.min);
            hi = hi.max(trace.max);
            values.extend(trace.runs.iter().flatten().map(|&(_, v)| v));
        }

        let mut clipped = None;
        let (mut y_lo, mut y_hi) = (lo, hi);

        if values.len() >= 200 {
            values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let at = |q: f64| values[((values.len() - 1) as f64 * q) as usize];
            let (p_lo, p_hi) = (at(0.003), at(0.997));
            let robust = p_hi - p_lo;
            // Six times is deliberately permissive. A rocket's altitude and
            // acceleration are *supposed* to have a huge dynamic range, and
            // clipping those would be the bug this is meant to prevent.
            if robust > 0.0 && (hi - lo) > robust * 6.0 {
                y_lo = p_lo;
                y_hi = p_hi;
                clipped = Some((lo, hi));
            }
        }

        if include_zero {
            y_lo = y_lo.min(0.0);
            y_hi = y_hi.max(0.0);
        }
        // A dead-flat trace has no range to scale to; give it one so the line
        // lands mid-panel instead of on an axis.
        if !(y_hi > y_lo) {
            let magnitude = y_hi.abs().max(1.0) * 0.05;
            y_lo -= magnitude;
            y_hi += magnitude;
        }
        let pad = (y_hi - y_lo) * 0.08;
        AxisRange {
            lo: y_lo - pad,
            hi: y_hi + pad,
            clipped,
        }
    }

    fn draw_panel(&self, area: &Area, panel: &Panel) -> Result<()> {
        area.fill(&theme::PANEL_BG).plot()?;
        let (w, _) = area.dim_in_pixel();
        // One reduction bucket per horizontal pixel of plotting area. Asking for
        // more would produce points that land on the same pixel.
        let buckets = (w as usize).saturating_sub(96).max(64);

        let traces: Vec<(&Line, Option<Trace>)> = panel
            .lines
            .iter()
            .map(|line| (line, self.trace(line, buckets)))
            .collect();

        if traces.iter().all(|(_, t)| t.is_none()) {
            return self.draw_absent_panel(area, panel, &traces);
        }

        let axis = self.axis_range(&traces, panel.zero_line);
        let (y_lo, y_hi) = (axis.lo as f64, axis.hi as f64);

        let mut chart = ChartBuilder::on(area)
            .caption(
                panel.title,
                TextStyle::from((theme::FONT, 16).into_font()).color(&theme::TEXT),
            )
            .margin_right(16)
            .margin_left(6)
            .margin_bottom(6)
            .x_label_area_size(28)
            .y_label_area_size(Y_GUTTER)
            .build_cartesian_2d(self.x_range.0..self.x_range.1, y_lo..y_hi)
            .plot()?;

        chart
            .configure_mesh()
            .light_line_style(theme::GRID.mix(0.45))
            .bold_line_style(theme::GRID)
            .axis_style(theme::AXIS)
            .label_style(TextStyle::from((theme::FONT, 12).into_font()).color(&theme::MUTED))
            .x_desc("")
            .y_desc(panel.unit)
            .axis_desc_style(TextStyle::from((theme::FONT, 12).into_font()).color(&theme::MUTED))
            .draw()
            .plot()?;

        self.draw_stage_bands(&mut chart, y_lo, y_hi)?;

        if panel.zero_line {
            chart
                .draw_series(std::iter::once(PathElement::new(
                    vec![(self.x_range.0, 0.0), (self.x_range.1, 0.0)],
                    theme::AXIS.stroke_width(1),
                )))
                .plot()?;
        }
        if panel.mark_apogee
            && let Some(t) = self.apogee_at_s()
        {
            chart
                .draw_series(DashedLineSeries::new(
                    vec![(t, y_lo), (t, y_hi)],
                    6,
                    4,
                    theme::MUTED.mix(0.75).stroke_width(1),
                ))
                .plot()?;
        }

        for (line, trace) in &traces {
            let Some(trace) = trace else { continue };
            for (i, run) in trace.runs.iter().enumerate() {
                let points: Vec<(f64, f64)> =
                    run.iter().map(|&(t, v)| (t, v as f64)).collect();
                let series = chart
                    .draw_series(LineSeries::new(points, line.color.stroke_width(1)))
                    .plot()?;
                // Only the first run carries the legend entry, or a gapped
                // trace would appear once per fragment.
                if i == 0 {
                    let color = line.color;
                    series.label(line.label).legend(move |(x, y)| {
                        PathElement::new(vec![(x, y), (x + 18, y)], color.stroke_width(3))
                    });
                }
            }
        }

        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(theme::BG.mix(0.82))
            .border_style(theme::AXIS)
            .label_font(TextStyle::from((theme::FONT, 12).into_font()).color(&theme::TEXT))
            .draw()
            .plot()?;

        // Footnotes for anything the panel is not showing at face value.
        let mut notes: Vec<String> = Vec::new();
        // Columns this log simply does not carry are named, so an empty-looking
        // panel is never ambiguous between "not fitted" and "nothing happened".
        let missing: Vec<&str> = traces
            .iter()
            .filter(|(_, t)| t.is_none())
            .map(|(l, _)| l.label)
            .collect();
        if !missing.is_empty() {
            notes.push(format!("no data: {}", missing.join(", ")));
        }
        if let Some((lo, hi)) = axis.clipped {
            notes.push(format!(
                "axis excludes outliers — full range {lo:.1} to {hi:.1} {}",
                panel.unit
            ));
        }
        if !notes.is_empty() {
            let (w, _) = area.dim_in_pixel();
            area.draw_text(
                &notes.join("   ·   "),
                &TextStyle::from((theme::FONT, 11).into_font())
                    .color(&theme::MUTED.mix(0.85))
                    .pos(Pos::new(HPos::Right, VPos::Center)),
                (w as i32 - 16, 14),
            )
            .plot()?;
        }
        Ok(())
    }

    /// A panel none of whose columns exist in this log.
    fn draw_absent_panel(
        &self,
        area: &Area,
        panel: &Panel,
        traces: &[(&Line, Option<Trace>)],
    ) -> Result<()> {
        let (w, h) = area.dim_in_pixel();
        // Same size and position as a live panel's caption, so a grid of panels
        // reads as one grid rather than as two kinds of thing.
        area.draw_text(
            panel.title,
            &TextStyle::from((theme::FONT, 16).into_font())
                .color(&theme::MUTED)
                .pos(Pos::new(HPos::Center, VPos::Center)),
            (w as i32 / 2, 14),
        )
        .plot()?;
        // Distinguishes a firmware that never wrote the column from one that
        // wrote it and had nothing to say — different problems, different fixes.
        let reason = if traces
            .iter()
            .all(|(l, _)| self.column(l.column).is_none())
        {
            "column not present in this log"
        } else {
            "nothing recorded during the flight"
        };
        area.draw_text(
            reason,
            &TextStyle::from((theme::FONT, 13).into_font())
                .color(&theme::MUTED.mix(0.7))
                .pos(Pos::new(HPos::Center, VPos::Center)),
            (w as i32 / 2, h as i32 / 2),
        )
        .plot()?;
        Ok(())
    }

    /// A strip of boolean lanes sharing the figure's time axis.
    ///
    /// Booleans get their own presentation rather than being drawn as 0/1 lines:
    /// six overlapping square waves on one y axis is unreadable, and what
    /// matters about a flag is *when* it was set, not its magnitude.
    fn draw_lanes(
        &self,
        area: &Area,
        title: &'static str,
        lanes: &[(&str, &str, RGBColor)],
    ) -> Result<()> {
        area.fill(&theme::PANEL_BG).plot()?;
        let n = lanes.len() as f64;

        let mut chart = ChartBuilder::on(area)
            .caption(
                title,
                TextStyle::from((theme::FONT, 16).into_font()).color(&theme::TEXT),
            )
            .margin_right(16)
            .margin_left(6)
            .margin_bottom(6)
            .x_label_area_size(28)
            .y_label_area_size(Y_GUTTER)
            .build_cartesian_2d(self.x_range.0..self.x_range.1, 0f64..n)
            .plot()?;

        chart
            .configure_mesh()
            .disable_y_mesh()
            .light_line_style(theme::GRID.mix(0.45))
            .bold_line_style(theme::GRID)
            .axis_style(theme::AXIS)
            .label_style(TextStyle::from((theme::FONT, 12).into_font()).color(&theme::MUTED))
            .y_labels(0)
            .x_desc("T+ seconds")
            .axis_desc_style(TextStyle::from((theme::FONT, 12).into_font()).color(&theme::MUTED))
            .draw()
            .plot()?;

        self.draw_stage_bands(&mut chart, 0.0, n)?;

        let label_style = TextStyle::from((theme::FONT, 11).into_font())
            .color(&theme::TEXT)
            .pos(Pos::new(HPos::Right, VPos::Center));
        // Bound rather than inlined: `color` borrows, and this style outlives
        // the statement that builds it.
        let dim = theme::MUTED.mix(0.55);
        let absent_style = TextStyle::from((theme::FONT, 11).into_font())
            .color(&dim)
            .pos(Pos::new(HPos::Right, VPos::Center));

        // Pixel geometry of the plotting area, so lane labels can be placed in
        // the y-label gutter at exactly the height of the lane they name.
        // `get_pixel_range` is in whole-backend coordinates, but `draw_text` on
        // this area is relative to the area's own origin. Without subtracting
        // the base, every label lands as far below the panel as the panel is
        // down the figure — off the image entirely for the lower rows.
        let (px, py) = chart.plotting_area().get_pixel_range();
        let (base_x, base_y) = area.get_base_pixel();
        let plot_top = (py.start - base_y) as f64;
        let plot_h = (py.end - py.start) as f64;
        let plot_w = (px.end - px.start) as f64;
        let label_right = px.start - base_x - 8;
        // A pulse a few rows long is well under a pixel wide at this scale.
        // Widening it to a floor keeps it visible; without this a pyro fire —
        // the shortest and most important event in the log — renders as nothing.
        let min_width = (self.x_range.1 - self.x_range.0) / plot_w.max(1.0) * 2.0;

        for (i, (label, column, color)) in lanes.iter().enumerate() {
            // Lane 0 at the top reads in the order the list is written.
            let top = n - i as f64;
            let bottom = top - 1.0;
            let inset = 0.24;

            let values = self.column(column);
            let present = values.is_some();
            let y_px = plot_top + plot_h * (i as f64 + 0.5) / n;
            area.draw_text(
                label,
                if present { &label_style } else { &absent_style },
                (label_right, y_px.round() as i32),
            )
            .plot()?;

            // The lane's own baseline, so an all-false flag still reads as
            // "recorded, never set" rather than as a blank row.
            if let Some(values) = values {
                chart
                    .draw_series(std::iter::once(PathElement::new(
                        vec![
                            (self.x_range.0, bottom + 0.5),
                            (self.x_range.1, bottom + 0.5),
                        ],
                        theme::GRID.stroke_width(1),
                    )))
                    .plot()?;

                let spans = true_spans(
                    &self.times,
                    values,
                    self.session.flight_start,
                    self.session.flight_end,
                );
                chart
                    .draw_series(spans.iter().map(|&(a, b)| {
                        Rectangle::new(
                            [
                                (a, bottom + inset),
                                (b.max(a + min_width), top - inset),
                            ],
                            color.mix(0.85).filled(),
                        )
                    }))
                    .plot()?;
            }
        }
        Ok(())
    }

    /// Wash the flight-stage bands in behind a chart's data.
    fn draw_stage_bands<DB: DrawingBackend>(
        &self,
        chart: &mut ChartContext<'_, DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
        y_lo: f64,
        y_hi: f64,
    ) -> Result<()>
    where
        DB::ErrorType: 'static,
    {
        let spans = self.stage_spans_in_window();
        chart
            .draw_series(spans.iter().map(|&(a, b, stage)| {
                Rectangle::new([(a, y_lo), (b, y_hi)], stage_color(stage).filled())
            }))
            .plot()?;
        Ok(())
    }
}
