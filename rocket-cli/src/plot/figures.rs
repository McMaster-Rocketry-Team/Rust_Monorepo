//! Rendering the three 3840×2160 figures.
//!
//! All three share one time ORIGIN: T+0 is ignition, and a few seconds of pad
//! sit in front of it so the ignition transient has something to be read
//! against — an axis that begins exactly at ignition shows the step but not
//! what it stepped from.
//!
//! They do not share a time RANGE. The air-brakes figure ends at apogee,
//! because the estimator behind every trace on it is retired there; the other
//! two run to landing. A point read off one figure is at the same T+ as on the
//! others, but the figures are not the same width in seconds, so they are read
//! by their labels rather than by laying them side by side.
//!
//! Altitudes are drawn AGL, converted from the ASL the log stores against the
//! pad reference it carries — AGL is the unit every threshold in the firmware
//! is configured in, so it is the unit these figures can be checked in.
//!
//! Within a figure the axis is shared in the stronger sense too: only the
//! bottom panel of a column carries tick labels. Repeating an identical row of
//! numbers under every panel spends about a tenth of the figure's height saying
//! the same thing five times, and that height is worth more given to the traces.
//! Vertical rules mark the events — burnout, apogee, deployments — and run
//! through every panel, which is what makes a feature in one panel locatable in
//! the next.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Result, anyhow};
use plotters::chart::SeriesAnno;
use plotters::coord::Shift;
use plotters::coord::types::RangedCoordf64;
use plotters::prelude::*;
use plotters::style::text_anchor::{HPos, Pos, VPos};

use crate::plot::events::{self, Event};
use crate::plot::log_csv::FlightLog;
use crate::plot::series::{Trace, decimate, stage_spans, true_spans};
use crate::plot::session::{Session, WindowSource};
use crate::plot::theme::{self, stage_color};

pub const WIDTH: u32 = 3840;
pub const HEIGHT: u32 = 2160;
/// Height reserved for the figure's own title block.
const HEADER_H: u32 = 150;
/// Width of the y-axis label gutter. One value for every panel on both figures:
/// panels share a time axis, and an axis that starts at a different x on each
/// row cannot be read across rows. It also has to be wide enough for the
/// longest lane name, which is why it is larger than the numbers alone need.
const Y_GUTTER: i32 = 232;
/// Height the x tick labels need, on the one panel per column that shows them.
const X_LABELS_H: u32 = 74;
/// Length of the colour swatch drawn beside each legend entry.
const LEGEND_SWATCH: i32 = 44;
/// Dash and gap lengths for a [`Line::dashed`] trace, in pixels.
const DASH: i32 = 26;
const DASH_GAP: i32 = 18;

type Area<'a> = DrawingArea<BitMapBackend<'a>, Shift>;

/// Label a drawn series, with a swatch that matches how it was drawn.
///
/// The dashed swatch is two strokes rather than one: a legend that shows a
/// solid rule against a dashed trace makes the reader match by colour alone,
/// which is exactly what the dashing was introduced to avoid needing.
fn legend_swatch<'a, DB: DrawingBackend + 'a>(series: &mut SeriesAnno<'a, DB>, line: &Line) {
    let color = line.color;
    if line.dashed {
        series.label(line.label).legend(move |(x, y)| {
            EmptyElement::at((x, y))
                + PathElement::new(vec![(0, 0), (18, 0)], color.stroke_width(6))
                + PathElement::new(
                    vec![(LEGEND_SWATCH - 18, 0), (LEGEND_SWATCH, 0)],
                    color.stroke_width(6),
                )
        });
    } else {
        series.label(line.label).legend(move |(x, y)| {
            EmptyElement::at((x, y))
                + PathElement::new(
                    vec![(0, 0), (LEGEND_SWATCH, 0)],
                    color.stroke_width(6),
                )
        });
    }
}

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
    /// Draw the trace broken rather than solid.
    ///
    /// Reserved for a value that is a *setting* rather than a measurement —
    /// the apogee the controller was aiming at is a constant somebody typed
    /// in, and drawing it in the same weight as the altitude it is being
    /// compared against invites reading it as another thing the rocket did.
    dashed: bool,
}

impl Line {
    fn new(label: &'static str, column: &'static str, color: RGBColor) -> Self {
        Self {
            label,
            column,
            color,
            scale: 1.0,
            dashed: false,
        }
    }

    fn scaled(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    fn dashed(mut self) -> Self {
        self.dashed = true;
        self
    }
}

/// A second y axis on the right of a panel.
///
/// Used where quantities that belong together are not measured in the same
/// thing — altitude against vertical acceleration, or vertical speed against
/// tilt and air-brake extension. The alternative is one panel each, which puts
/// the very comparison the reader wants on two different time axes.
struct Secondary {
    unit: &'static str,
    lines: Vec<Line>,
}

impl Secondary {
    fn new(unit: &'static str, lines: Vec<Line>) -> Self {
        Self { unit, lines }
    }
}

struct Panel {
    title: &'static str,
    unit: &'static str,
    lines: Vec<Line>,
    secondary: Option<Secondary>,
    /// Draw a reference line at zero. Only meaningful where the sign carries
    /// information — velocity crossing zero *is* apogee.
    zero_line: bool,
    /// Bottom of a column: carries the tick labels for everything above it.
    x_labels: bool,
    /// Top of a column: carries the names of the event rules.
    event_labels: bool,
}

impl Panel {
    fn new(title: &'static str, unit: &'static str, lines: Vec<Line>) -> Self {
        Self {
            title,
            unit,
            lines,
            secondary: None,
            zero_line: false,
            x_labels: false,
            event_labels: false,
        }
    }

    fn with_secondary(mut self, secondary: Secondary) -> Self {
        self.secondary = Some(secondary);
        self
    }

    fn with_zero(mut self) -> Self {
        self.zero_line = true;
        self
    }

    fn with_x_labels(mut self) -> Self {
        self.x_labels = true;
        self
    }

    fn with_event_labels(mut self) -> Self {
        self.event_labels = true;
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

/// The rows one figure draws, as a half-open range into the log.
///
/// A figure-level rather than a session-level property, because the three
/// figures deliberately do not cover the same stretch of flight: the airbrakes
/// story is over at apogee and drawing the descent beside it would spend half
/// the width on a window where every trace on the figure is absent.
#[derive(Debug, Clone, Copy)]
pub struct Window {
    pub start: usize,
    pub end: usize,
}

/// A vertical rule drawn through every panel.
///
/// Two kinds share the presentation because they answer the same question —
/// "what changed here?" — but they are styled apart on purpose: the flight
/// events are things that happened to the rocket, and the brakes-permission
/// divider is a decision the software made. Reading the second as the first
/// would be reading a permission as an occurrence.
struct Rule {
    at_s: f64,
    label: String,
    color: RGBColor,
    /// Dotted rather than dashed, and drawn opaque rather than washed out.
    dotted: bool,
}

pub struct Renderer<'a> {
    log: &'a FlightLog,
    session: &'a Session,
    /// Seconds relative to liftoff, per row of the whole log.
    times: Vec<f64>,
    /// Columns computed rather than logged.
    derived: HashMap<&'static str, Vec<f32>>,
    /// `(start, end)` rows this figure draws. Everything that reduces a column
    /// or sizes an axis reads it, so a figure cannot accidentally draw outside
    /// the window it advertised.
    window: (usize, usize),
    /// The apogee the flight actually reached, above the pad. `None` when no
    /// estimator reported one, or when the log carries no pad reference to
    /// measure it from.
    apogee_agl: Option<f32>,
    /// `(ignition, burnout)` in figure seconds — the stretch the motor was
    /// driving the airframe. `None` when the log carries no burnout flag, or
    /// when it never went true.
    burn_span: Option<(f64, f64)>,
    /// Every stretch the brakes were permitted to open, in figure seconds.
    ///
    /// Plural, and not simply "from the first one onwards": the gate is
    /// re-evaluated every sample, and a coast that dips back above the Mach
    /// limit — or a barometer that loses trust — closes it again. Drawing it
    /// as spans is what makes that visible instead of assumed.
    brakes_spans: Vec<(f64, f64)>,
    x_range: (f64, f64),
    events: Vec<Event>,
    source_name: String,
}

impl<'a> Renderer<'a> {
    /// One renderer per figure — `window` is what distinguishes them.
    ///
    /// The derived columns below are recomputed per figure rather than shared.
    /// They are two linear passes over the log and cost microseconds, and
    /// paying them three times is cheaper than the plumbing that would let
    /// three windows share one buffer.
    pub fn new(
        log: &'a FlightLog,
        session: &'a Session,
        source_name: String,
        window: Window,
    ) -> Self {
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
            let magnitude: Vec<f32> = (0..log.row_count)
                .map(|i| (x[i] * x[i] + y[i] * y[i] + z[i] * z[i]).sqrt())
                .collect();
            derived.insert("acc_magnitude", magnitude);
        }

        // Vertical acceleration in the earth frame. The log carries only
        // body-frame components, so the longitudinal axis is projected onto the
        // vertical with the estimator's own tilt and gravity is removed:
        //
        //     a_vertical = a_z * cos(tilt) - g
        //
        // On the pad that is 9.81 * cos(0) - 9.81 = 0, which is the check that
        // the sign convention is right. Lateral components are left out — they
        // are under 0.05 m/s^2 through this flight, far below the error the
        // tilt estimate itself carries.
        //
        // It is deliberately absent wherever tilt is, rather than assuming
        // vertical: under drogue the airframe hangs at an angle, and pretending
        // otherwise would report a fabricated acceleration during exactly the
        // phase where the assumption fails.
        if let (Some(z), Some(tilt)) = (
            log.column("acc_z"),
            log.column("airbrakes_kf_tilt_deg"),
        ) {
            const G: f32 = 9.80665;
            let vertical = (0..log.row_count)
                .map(|i| {
                    if tilt[i].is_finite() && z[i].is_finite() {
                        z[i] * tilt[i].to_radians().cos() - G
                    } else {
                        f32::NAN
                    }
                })
                .collect();
            derived.insert("vertical_acc_earth", vertical);
        }

        // Every altitude on the card is ASL, which is the unit the barometer
        // and the GPS measure in and the only one that needs no reference to
        // interpret. Every altitude a reader of these figures cares about is
        // AGL — it is what the deployment thresholds, the apogee target and
        // the flight itself were specified in — so the conversion happens
        // here, once, against the pad reference the log now carries.
        //
        // Row-wise rather than against a single scalar: on the pad the
        // reference is a settling low pass, and subtracting it row by row is
        // what puts the pre-launch trace at 0 instead of at whatever the low
        // pass had reached. Rows before the first slow record have no
        // reference of their own and fall back to the first one the session
        // ever recorded, so the conversion leaves no gap that the source
        // column did not already have.
        let pad_asl = log.column("launch_pad_altitude_asl");
        let pad_fallback = pad_asl
            .and_then(|pad| pad[session.start..session.end].iter().find(|v| v.is_finite()).copied());
        if let Some(pad) = pad_asl {
            for (asl_name, agl_name) in [
                ("deployment_kf_altitude_asl", "deployment_kf_altitude_agl"),
                ("airbrakes_kf_altitude_asl", "airbrakes_kf_altitude_agl"),
                ("gps_altitude_asl", "gps_altitude_agl"),
                ("mpc_predicted_apogee_asl", "mpc_predicted_apogee_agl"),
                ("air_brakes_target_apogee_asl", "air_brakes_target_apogee_agl"),
            ] {
                let Some(asl) = log.column(asl_name) else {
                    continue;
                };
                let agl = (0..log.row_count)
                    .map(|i| {
                        let reference = if pad[i].is_finite() {
                            Some(pad[i])
                        } else {
                            pad_fallback
                        };
                        match reference {
                            Some(r) if asl[i].is_finite() => asl[i] - r,
                            _ => f32::NAN,
                        }
                    })
                    .collect();
                derived.insert(agl_name, agl);
            }
        }

        // The apogee the flight actually reached, above the pad — the number
        // the whole airbrakes controller was trying to place, and the one the
        // header quotes.
        let apogee_agl = match (session.apogee_asl, session.apogee_row) {
            (Some(asl), Some(row)) => {
                let reference = pad_asl
                    .map(|pad| pad[row])
                    .filter(|v| v.is_finite())
                    .or(pad_fallback);
                reference.map(|r| asl - r)
            }
            _ => None,
        };

        // T+0 stays ignition; the axis simply starts at a negative number when
        // there is a lead-in.
        let x_range = (
            times[window.start],
            times[window.end - 1].max(times[window.start] + 0.001),
        );

        // The motor burn, as a span rather than as the two rules bounding it.
        // It opens at T+0 by definition — the flight-start row IS the ignition
        // detection — and closes on the first row the estimator declares
        // burnout. Left `None` rather than guessed at when the flag is absent:
        // a shaded region that says "the motor was lit here" has to be sourced
        // from the flag that decided it, not from a plausible duration.
        let burn_span = log.column("airbrakes_burnout").and_then(|flag| {
            (session.flight_start..window.end)
                .find(|&i| flag[i] >= 0.5)
                .map(|row| (times[session.flight_start], times[row]))
        });

        // Straight off the estimator's logged state — the brakes are
        // permitted in exactly one of its four, and that is a decision the
        // log records rather than one this tool re-derives. Re-deriving it
        // is not even possible: the Mach test behind the transition needs a
        // config constant the log does not carry.
        let brakes_flag: Vec<f32> = log
            .airbrakes_state
            .iter()
            .map(|state| match state {
                Some(3) => 1.0,
                Some(_) => 0.0,
                None => f32::NAN,
            })
            .collect();
        let brakes_spans = true_spans(&times, &brakes_flag, window.start, window.end);

        // Half a percent of the flight. Two events closer together than that
        // cannot be told apart on the axis, so they are drawn as one.
        let merge_within = (x_range.1 - x_range.0) * 0.005;
        let events = events::detect(
            &times,
            &log.stage,
            log.column("airbrakes_burnout"),
            log.column("pyro_drogue_fire"),
            log.column("pyro_main_fire"),
            session.apogee_row,
            window.start,
            window.end,
            merge_within,
        );

        Self {
            log,
            session,
            times,
            derived,
            window: (window.start, window.end),
            apogee_agl,
            burn_span,
            brakes_spans,
            x_range,
            events,
            source_name,
        }
    }

    /// Every vertical rule on this figure, in time order.
    ///
    /// The brakes divider is the first sample the gate opened on, and only the
    /// first: the gate can close and reopen during the coast, and a rule per
    /// edge would bury the one moment that matters — when the controller was
    /// first allowed to act — under its own noise. The reopenings are still on
    /// the figure, as the background band starting again.
    fn rules(&self) -> Vec<Rule> {
        let mut rules: Vec<Rule> = self
            .events
            .iter()
            .map(|event| Rule {
                at_s: event.at_s,
                label: event.label.clone(),
                color: theme::EVENT,
                dotted: false,
            })
            .collect();
        if let Some(&(at_s, _)) = self.brakes_spans.first() {
            rules.push(Rule {
                at_s,
                label: "brakes enabled".to_string(),
                color: theme::BRAKES_HUE,
                dotted: true,
            });
        }
        rules.sort_by(|a, b| a.at_s.partial_cmp(&b.at_s).unwrap_or(std::cmp::Ordering::Equal));
        rules
    }

    /// Apogee on this figure's axis — seconds after ignition.
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
            self.window.0,
            self.window.1,
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

    /// The air brakes, from the pad to apogee.
    ///
    /// The window ends at apogee because the controller does: the airbrakes
    /// estimator is retired on the sample its vertical velocity reaches zero,
    /// so every trace on this figure goes absent there at once. Carrying the
    /// axis to landing would spend more than half the width drawing the gap
    /// after them.
    ///
    /// Two panels, read top to bottom as one argument: where the rocket was
    /// going and what the controller did about it, and the vertical state that
    /// drove the decision.
    ///
    /// The estimator's flag strip used to be a third panel and is gone. The
    /// two flags that describe the *shape* of the flight — the burn and the
    /// brakes-permission window — are background bands on both panels now,
    /// which is where a reader wants them: behind the traces they qualify,
    /// rather than in a strip that has to be read against them. The rest were
    /// per-sample diagnostics that answer "why did this not happen", a
    /// question worth its own look at the CSV rather than a permanent fifth of
    /// this figure's height.
    pub fn render_airbrakes(&self, path: &Path) -> Result<()> {
        let root = BitMapBackend::new(path, (WIDTH, HEIGHT)).into_drawing_area();
        root.fill(&theme::BG).plot()?;
        let (header, body) = root.split_vertically(HEADER_H);
        self.draw_header(&header, "Air brakes")?;

        // Equal halves. The two panels are read against each other — where the
        // speed trace bends is where the altitude trace flattens — and equal
        // heights are what keep a slope on one comparable to a slope on the
        // other.
        let (p1, p2) = body.split_vertically((HEIGHT - HEADER_H) / 2);

        self.draw_panel(
            &p1,
            &Panel::new(
                "Altitude, apogee prediction & tilt",
                "m AGL",
                vec![
                    // Three numbers in one reference, which is the point of the
                    // panel: the altitude climbing, the apogee it is predicted
                    // to reach from here, and the apogee being aimed at. A gap
                    // between the prediction and the target is the controller
                    // saying it cannot get there; the two converging is it
                    // saying it can.
                    Line::new("airbrakes KF altitude", "airbrakes_kf_altitude_agl", theme::CYAN),
                    Line::new("MPC predicted apogee", "mpc_predicted_apogee_agl", theme::AMBER),
                    Line::new("target apogee", "air_brakes_target_apogee_agl", theme::VIOLET)
                        .dashed(),
                ],
            )
            .with_event_labels()
            // Tilt belongs against the altitude rather than against the speed:
            // what it explains is why the prediction and the target diverge —
            // a rocket leaning over is one that will not reach the apogee a
            // vertical flight would.
            .with_secondary(Secondary::new(
                "°",
                vec![Line::new("tilt", "airbrakes_kf_tilt_deg", theme::ROSE)],
            )),
            Y_GUTTER,
        )?;
        self.draw_panel(
            &p2,
            &Panel::new(
                // Speed and acceleration share the left axis rather than being
                // split across two: one is the derivative of the other, they
                // are within a factor of three of each other on this flight,
                // and the question the panel exists to answer — did the speed
                // start falling when the acceleration went negative — is a
                // question about where two curves cross, which needs them on
                // one grid. The extension rides the right axis of the same
                // panel because it is the cause and those two are the effect:
                // the brakes coming out is what bends the acceleration down.
                "Vertical speed, acceleration & brake extension",
                "m/s · m/s²",
                vec![
                    Line::new(
                        "vertical speed (airbrakes KF)",
                        "airbrakes_kf_vertical_velocity",
                        theme::CYAN,
                    ),
                    Line::new(
                        "vertical acceleration (earth)",
                        "vertical_acc_earth",
                        theme::GREEN,
                    ),
                ],
            )
            // Zero means two different things on this panel and both are
            // worth a rule: velocity crossing it is apogee, acceleration
            // crossing it is the top of the drag-only coast.
            .with_zero()
            .with_x_labels()
            .with_secondary(Secondary::new(
                "%",
                vec![
                    // Warm pair against the cool pair on the left, so which
                    // axis a trace belongs to is readable without chasing it
                    // to the legend. Commanded is not green here — that is the
                    // acceleration on this panel now.
                    Line::new("brakes commanded", "air_brakes_commanded_extension", theme::AMBER)
                        .scaled(100.0),
                    Line::new("brakes actual", "air_brakes_actual_extension", theme::ROSE)
                        .scaled(100.0),
                ],
            )),
            Y_GUTTER,
        )?;

        root.present().plot()?;
        Ok(())
    }

    /// The deployment estimator and the pyros, pad to landing.
    ///
    /// This is the half of the avionics that runs the whole flight and fires
    /// the charges, so its window is the whole flight — the descent is not
    /// padding here, it is where two of the three panels do their work.
    ///
    /// Altitude is AGL because every threshold this estimator acts on is: the
    /// drogue's minimum altitude and the main's deployment altitude are both
    /// configured above ground, and a figure that made the reader subtract a
    /// pad altitude before checking them against the pyro lanes would be
    /// hiding the one comparison it exists to support.
    pub fn render_deployment(&self, path: &Path) -> Result<()> {
        let root = BitMapBackend::new(path, (WIDTH, HEIGHT)).into_drawing_area();
        root.fill(&theme::BG).plot()?;
        let (header, body) = root.split_vertically(HEADER_H);
        self.draw_header(&header, "Deployment")?;

        let (p1, rest) = body.split_vertically(820);
        let (p2, p3) = rest.split_vertically(600);

        self.draw_panel(
            &p1,
            &Panel::new(
                "Altitude",
                "m AGL",
                vec![
                    // The GPS rides along as the independent witness: it is the
                    // only altitude on the figure that does not come from the
                    // barometer, so a baro that drifted or iced shows up as the
                    // two traces separating rather than as a plausible curve.
                    Line::new("deployment KF", "deployment_kf_altitude_agl", theme::CYAN),
                    Line::new("GPS", "gps_altitude_agl", theme::VIOLET),
                ],
            )
            .with_event_labels()
            .with_zero(),
            Y_GUTTER,
        )?;
        self.draw_panel(
            &p2,
            &Panel::new(
                "Vertical speed",
                "m/s",
                vec![Line::new(
                    "deployment KF",
                    "deployment_kf_vertical_velocity",
                    theme::CYAN,
                )],
            )
            // Zero is apogee, and the two descent rates either side of the main
            // are read off this panel against it.
            .with_zero(),
            Y_GUTTER,
        )?;
        self.draw_lanes(
            &p3,
            "Pyro & deployment baro gate",
            &[
                ("drogue cont.", "pyro_drogue_continuity", theme::CYAN),
                ("drogue fire", "pyro_drogue_fire", theme::AMBER),
                ("main cont.", "pyro_main_continuity", theme::CYAN),
                ("main fire", "pyro_main_fire", theme::AMBER),
                ("short circuit", "pyro_short_circuit", theme::ALERT),
                // The filter that fired those charges, and what it made of its
                // own barometer at the time. "Dropped" rather than "rejected"
                // because the bundled font clips the descender of a `j` at
                // this size, and a lane label is too small to lose a glyph in.
                ("gate dropped", "deployment_baro_gate_reject", theme::ALERT),
                ("baro resync", "deployment_baro_resync", theme::ROSE),
            ],
            true,
            Y_GUTTER,
        )?;

        root.present().plot()?;
        Ok(())
    }

    /// Everything else that was logged, pad to landing.
    ///
    /// The raw sensors, the environment, power, GPS quality, the payload rails
    /// and the CAN bus. A column that is logged and plotted nowhere is a column
    /// nobody will remember to look at, so this figure is deliberately a
    /// catch-all rather than a curated view.
    pub fn render_misc(&self, path: &Path) -> Result<()> {
        let root = BitMapBackend::new(path, (WIDTH, HEIGHT)).into_drawing_area();
        root.fill(&theme::BG).plot()?;
        let (header, body) = root.split_vertically(HEADER_H);
        self.draw_header(&header, "Auxiliary")?;

        // Four rows of three. Only the bottom row carries tick labels, so it is
        // the only row that has to be taller.
        let (top, bottom) = body.split_vertically(HEIGHT - HEADER_H - 470);
        let upper = top.split_evenly((3, 3));
        let lower = bottom.split_evenly((1, 3));
        let cells: Vec<_> = upper.into_iter().chain(lower).collect();

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
            .with_zero()
            .with_event_labels(),
            Panel::new(
                "Acceleration (body frame)",
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
            .with_zero()
            .with_event_labels(),
            Panel::new(
                "Magnetometer",
                "gauss",
                vec![
                    Line::new("x", "mag_x", theme::CORAL),
                    Line::new("y", "mag_y", theme::BLUE),
                    Line::new("z", "mag_z", theme::ROSE),
                ],
            )
            .with_zero()
            .with_event_labels(),
            Panel::new(
                "Barometric pressure",
                "Pa",
                vec![Line::new("baro", "pressure", theme::CYAN)],
            ),
            Panel::new(
                // The reference the other two figures subtract. Flat for the
                // whole flight by construction — it is latched at ignition —
                // so what this panel is really for is the pad segment in front
                // of T+0, where the low pass is still settling, and for reading
                // any AGL number on the other figures back to ASL.
                "Launch pad altitude (AGL reference)",
                "m ASL",
                vec![
                    Line::new("pad reference", "launch_pad_altitude_asl", theme::GREEN),
                    Line::new("GPS", "gps_altitude_asl", theme::VIOLET),
                ],
            ),
            Panel::new(
                "Tilt from vertical",
                "°",
                vec![Line::new("airbrakes KF", "airbrakes_kf_tilt_deg", theme::VIOLET)],
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
            // Bottom row: these carry the shared tick labels.
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
            )
            .with_x_labels(),
            Panel::new(
                "Payload actuators",
                "steps",
                vec![
                    Line::new("actuator 1", "payload_actuator_1_steps", theme::CYAN),
                    Line::new("actuator 2", "payload_actuator_2_steps", theme::AMBER),
                    Line::new("actuator 3", "payload_actuator_3_steps", theme::GREEN),
                ],
            )
            .with_x_labels(),
            Panel::new(
                "CAN node uptime",
                "s",
                vec![
                    Line::new("AMP", "amp_uptime_s", theme::CYAN),
                    Line::new("ICARUS", "icarus_uptime_s", theme::AMBER),
                    Line::new("OZYS", "ozys_uptime_s", theme::GREEN),
                    Line::new("payload SDRM", "payload_sdrm_uptime_s", theme::VIOLET),
                ],
            )
            .with_x_labels(),
        ];

        for (cell, panel) in cells.iter().zip(panels.iter()) {
            self.draw_panel(cell, panel, 0)?;
        }

        root.present().plot()?;
        Ok(())
    }

    // ---------------------------------------------------------------- pieces

    fn draw_header(&self, area: &Area, kind: &str) -> Result<()> {
        area.fill(&theme::BG).plot()?;
        let title = TextStyle::from((theme::FONT, theme::F_TITLE).into_font())
            .color(&theme::TEXT)
            .pos(Pos::new(HPos::Left, VPos::Center));
        let sub = TextStyle::from((theme::FONT, theme::F_SUBTITLE).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Left, VPos::Center));
        let right = TextStyle::from((theme::FONT, theme::F_SUBTITLE).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Right, VPos::Center));

        area.draw_text(&format!("{kind} · {}", self.source_name), &title, (56, 54))
            .plot()?;

        let s = self.session;
        // AGL, to match every altitude axis on the three figures. The ASL it
        // was measured as rides along in parentheses, because that is the
        // number the log actually stores and the one to quote when comparing
        // against a GPS or a simulation.
        let apogee = match (self.apogee_agl, s.apogee_asl, self.apogee_at_s()) {
            (Some(agl), Some(asl), Some(t)) => {
                format!("apogee {agl:.0} m AGL ({asl:.0} m ASL) at T+{t:.1} s")
            }
            (None, Some(asl), Some(t)) => format!("apogee {asl:.0} m ASL at T+{t:.1} s"),
            _ => "apogee not recorded".to_string(),
        };
        area.draw_text(
            &format!(
                "T+0 = ignition · {:.1} s of flight · {} rows · {}",
                s.duration_s(self.log),
                s.flight_rows(),
                apogee
            ),
            &sub,
            (56, 106),
        )
        .plot()?;

        // The trim is stated rather than assumed. A reader who wonders where the
        // pad time went should not have to guess that it was removed.
        let trim = match s.window_source {
            WindowSource::Stages => format!(
                "from T-{:.0} s · trimmed {:.1} s on the pad and {:.1} s after landing",
                s.lead_in_s(self.log),
                s.trimmed_before_s(self.log),
                s.trimmed_after_s(self.log)
            ),
            WindowSource::StagesNoLanding => format!(
                "from T-{:.0} s · trimmed {:.1} s on the pad · log ends before landing",
                s.lead_in_s(self.log),
                s.trimmed_before_s(self.log)
            ),
            WindowSource::NeverLeftThePad => {
                "never left the pad · showing the whole session".to_string()
            }
        };
        area.draw_text(&trim, &right, (WIDTH as i32 - 56, 54))
            .plot()?;
        if let Some(failed) = self.log.crc_failed_rows.filter(|n| *n > 0) {
            let warn = TextStyle::from((theme::FONT, theme::F_SUBTITLE).into_font())
                .color(&theme::ALERT)
                .pos(Pos::new(HPos::Right, VPos::Center));
            area.draw_text(
                &format!("{failed} row(s) from CRC-failed blocks"),
                &warn,
                (WIDTH as i32 - 56, 106),
            )
            .plot()?;
        }
        self.draw_stage_key(area)?;
        Ok(())
    }

    /// Name the background washes, in the order they occurred.
    ///
    /// Every panel is washed with these, and an unlabelled wash is just a stain.
    /// The chip is drawn as panel background plus the same translucent fill the
    /// bands use, over an opaque rule in the band's hue — so the swatch is
    /// literally what the reader sees behind the traces, with enough edge to
    /// find it.
    ///
    /// The burn joins the stages here rather than being explained in a caption,
    /// because on the figure it is exactly the same kind of mark: a coloured
    /// region behind the traces, which the reader has to be able to name.
    fn draw_stage_key(&self, area: &Area) -> Result<()> {
        let mut entries: Vec<(&str, RGBColor, RGBAColor)> = Vec::new();
        let mut stages: Vec<u8> = Vec::new();
        for (_, _, stage) in self.stage_spans_in_window() {
            if !stages.contains(&stage) {
                stages.push(stage);
                entries.push((
                    theme::stage_name(stage),
                    theme::stage_hue(stage),
                    stage_color(stage),
                ));
            }
        }
        // Last, because it is the layer drawn last on the panels too, and the
        // key reads in the order the colours are stacked.
        if self.burn_span.is_some() {
            entries.push(("Burn", theme::BURN_HUE, theme::burn_color()));
        }
        if !self.brakes_spans.is_empty() {
            entries.push((
                "Brakes enabled",
                theme::BRAKES_HUE,
                theme::brakes_color(),
            ));
        }
        if entries.is_empty() {
            return Ok(());
        }

        const CHIP_W: i32 = 54;
        const CHIP_H: i32 = 28;
        const GAP: i32 = 26;
        // ~0.52 em per character is close enough to centre the row; being a few
        // pixels off is invisible, and measuring text would mean rendering it
        // twice.
        let widths: Vec<i32> = entries
            .iter()
            .map(|(label, _, _)| {
                CHIP_W + 10 + (label.len() as i32 * theme::F_SUBTITLE * 52) / 100 + GAP
            })
            .collect();
        let total: i32 = widths.iter().sum::<i32>() - GAP;

        let mut x = (WIDTH as i32 - total) / 2;
        let y = 80;
        let text = TextStyle::from((theme::FONT, theme::F_SUBTITLE).into_font())
            .color(&theme::MUTED)
            .pos(Pos::new(HPos::Left, VPos::Center));

        for ((label, hue, wash), width) in entries.iter().zip(widths) {
            let chip = [(x, y - CHIP_H / 2), (x + CHIP_W, y + CHIP_H / 2)];
            area.draw(&Rectangle::new(chip, theme::PANEL_BG.filled()))
                .plot()?;
            area.draw(&Rectangle::new(chip, wash.filled()))
                .plot()?;
            area.draw(&Rectangle::new(
                [(x, y + CHIP_H / 2), (x + CHIP_W, y + CHIP_H / 2 + 4)],
                hue.filled(),
            ))
            .plot()?;
            area.draw_text(label, &text, (x + CHIP_W + 10, y))
                .plot()?;
            x += width;
        }
        Ok(())
    }

    fn stage_spans_in_window(&self) -> Vec<(f64, f64, u8)> {
        stage_spans(
            &self.times,
            &self.log.stage,
            self.window.0,
            self.window.1,
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

    fn draw_panel(&self, area: &Area, panel: &Panel, right_gutter: i32) -> Result<()> {
        area.fill(&theme::PANEL_BG).plot()?;
        let (w, _) = area.dim_in_pixel();
        // One reduction bucket per horizontal pixel of plotting area. Asking for
        // more would produce points that land on the same pixel.
        let buckets = (w as usize).saturating_sub(Y_GUTTER as usize).max(64);

        let traces: Vec<(&Line, Option<Trace>)> = panel
            .lines
            .iter()
            .map(|line| (line, self.trace(line, buckets)))
            .collect();

        let nothing_primary = traces.iter().all(|(_, t)| t.is_none());
        let nothing_secondary = panel.secondary.as_ref().is_none_or(|sec| {
            sec.lines
                .iter()
                .all(|line| self.trace(line, buckets).is_none())
        });
        if nothing_primary && nothing_secondary {
            return self.draw_absent_panel(area, panel, &traces, right_gutter);
        }

        let axis = self.axis_range(&traces, panel.zero_line);
        let (y_lo, y_hi) = (axis.lo as f64, axis.hi as f64);

        // The secondary traces are reduced up front so its axis can be sized
        // before the chart is built.
        let secondary: Vec<(&Line, Option<Trace>)> = panel
            .secondary
            .iter()
            .flat_map(|s| s.lines.iter())
            .map(|line| (line, self.trace(line, buckets)))
            .collect();
        // Never forced through zero, unlike the primary: the right-hand axis
        // carries whatever did not fit on the left, and there is no reason its
        // zero should be meaningful. Tilt is the case in point — it cannot be
        // negative, and pinning it to zero would waste half the panel.
        let s_axis = panel
            .secondary
            .as_ref()
            .map(|_| self.axis_range(&secondary, false));
        let (s_lo, s_hi) = s_axis
            .as_ref()
            .map(|a| (a.lo as f64, a.hi as f64))
            .unwrap_or((0.0, 1.0));

        let mut chart = ChartBuilder::on(area)
            .caption(
                panel.title,
                TextStyle::from((theme::FONT, theme::F_CAPTION).into_font()).color(&theme::TEXT),
            )
            .margin_right(if right_gutter > 0 { 14 } else { 40 })
            .margin_left(14)
            .margin_bottom(12)
            .x_label_area_size(if panel.x_labels { X_LABELS_H } else { 0 })
            .y_label_area_size(Y_GUTTER)
            .right_y_label_area_size(right_gutter)
            .build_cartesian_2d(self.x_range.0..self.x_range.1, y_lo..y_hi)
            .plot()?
            // Always dual, even with nothing on the right: the gutter is
            // reserved per figure so that panels sharing a time axis start and
            // end at the same x, and a panel that opted out of the second axis
            // must not therefore be wider than its neighbours.
            .set_secondary_coord(self.x_range.0..self.x_range.1, s_lo..s_hi);

        {
            let mut binding = chart.configure_mesh();
            let mesh = binding
                .light_line_style(theme::GRID.mix(0.55))
                .bold_line_style(theme::GRID)
                .axis_style(theme::AXIS)
                .label_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                )
                .y_desc(panel.unit)
                .axis_desc_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                );
            // The gridlines stay on every panel; only the numbers under them are
            // dropped, which is the whole saving.
            if panel.x_labels {
                mesh.x_desc("T+ seconds");
            } else {
                mesh.disable_x_axis();
            }
            mesh.draw().plot()?;
        }

        if let Some(sec) = &panel.secondary {
            chart
                .configure_secondary_axes()
                .y_desc(sec.unit)
                .axis_style(theme::AXIS)
                .label_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                )
                .axis_desc_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                )
                .draw()
                .plot()?;
        }

        self.draw_stage_bands(&mut chart, y_lo, y_hi)?;
        self.draw_event_rules(&mut chart, y_lo, y_hi)?;
        self.draw_plot_border(&mut chart, y_lo, y_hi)?;

        if panel.zero_line {
            chart
                .draw_series(std::iter::once(PathElement::new(
                    vec![(self.x_range.0, 0.0), (self.x_range.1, 0.0)],
                    theme::AXIS.stroke_width(2),
                )))
                .plot()?;
        }

        for (line, trace) in &traces {
            let Some(trace) = trace else { continue };
            for (i, run) in trace.runs.iter().enumerate() {
                let points: Vec<(f64, f64)> =
                    run.iter().map(|&(t, v)| (t, v as f64)).collect();
                // Drawn segment by segment rather than as one `LineSeries`.
                //
                // plotters builds a thick polyline by joining quads, and at a
                // sharp enough cusp the join projects a miter spike far outside
                // the plotting area — on this data the temperature trace, which
                // min/max decimation turns into a picket fence of one-bucket
                // spikes, threw them up through the panel above and read as
                // stray data in someone else's chart. A two-point path has no
                // join to compute, so the failure cannot arise. At 2 px the
                // missing joins are not visible.
                let style = line.color.stroke_width(if line.dashed { 3 } else { 2 });
                let series = if line.dashed {
                    chart
                        .draw_series(DashedLineSeries::new(
                            points.iter().copied(),
                            DASH,
                            DASH_GAP,
                            style,
                        ))
                        .plot()?
                } else {
                    chart
                        .draw_series(
                            points
                                .windows(2)
                                .map(|w| PathElement::new(vec![w[0], w[1]], style)),
                        )
                        .plot()?
                };
                // Only the first run carries the legend entry, or a gapped
                // trace would appear once per fragment.
                if i == 0 {
                    legend_swatch(series, line);
                }
            }
        }

        for (line, trace) in &secondary {
            let Some(trace) = trace else { continue };
            for (i, run) in trace.runs.iter().enumerate() {
                let points: Vec<(f64, f64)> =
                    run.iter().map(|&(t, v)| (t, v as f64)).collect();
                let style = line.color.stroke_width(if line.dashed { 3 } else { 2 });
                let series = if line.dashed {
                    chart
                        .draw_secondary_series(DashedLineSeries::new(
                            points.iter().copied(),
                            DASH,
                            DASH_GAP,
                            style,
                        ))
                        .plot()?
                } else {
                    chart
                        .draw_secondary_series(
                            points
                                .windows(2)
                                .map(|w| PathElement::new(vec![w[0], w[1]], style)),
                        )
                        .plot()?
                };
                // `draw_secondary_series` hands back an annotation on the
                // *primary* chart, so one legend covers both axes.
                if i == 0 {
                    legend_swatch(series, line);
                }
            }
        }

        if panel.event_labels {
            // Measured, not guessed at, because the event labels have to dodge
            // it: the legend moved to the upper left, which is also where the
            // labels start, and on the auxiliary figure every event of the
            // flight falls inside the first tenth of a 500 s axis — directly
            // under it.
            //
            // plotters does not report the box it drew, so this reproduces its
            // sizing: one row per labelled series, at the swatch gutter plus
            // ~0.52 em per character. Being a little generous is free — the
            // cost of overestimating is a label one row lower than it had to
            // be, and of underestimating is two texts on top of each other.
            let rows = traces
                .iter()
                .chain(secondary.iter())
                .filter(|(_, t)| t.is_some())
                .count();
            let widest = traces
                .iter()
                .chain(secondary.iter())
                .filter(|(_, t)| t.is_some())
                .map(|(l, _)| l.label.chars().count())
                .max()
                .unwrap_or(0);
            let legend = (rows > 0).then(|| {
                (
                    (LEGEND_SWATCH + 18) as f64
                        + widest as f64 * theme::F_LEGEND as f64 * 0.52
                        + 30.0,
                    rows as f64 * (theme::F_LEGEND as f64 + 12.0) + 24.0,
                )
            });
            self.draw_event_labels(&mut chart, y_lo, y_hi, legend)?;
        }

        chart
            .configure_series_labels()
            // Top left, where the eye starts. It is also the corner the data
            // is least likely to be in on these figures: every trace here
            // begins on the pad at rest, so the upper left is the one region
            // that is empty by physics rather than by luck.
            .position(SeriesLabelPosition::UpperLeft)
            // plotters sizes the swatch gutter from this, not from the element
            // the closure draws; leaving it at the default puts the last few
            // pixels of every swatch through the first letter of its label.
            .legend_area_size(LEGEND_SWATCH as u32 + 18)
            .background_style(theme::PANEL_BG.mix(0.88))
            .border_style(theme::GRID)
            .label_font(
                TextStyle::from((theme::FONT, theme::F_LEGEND).into_font()).color(&theme::TEXT),
            )
            .draw()
            .plot()?;

        // Footnotes for anything the panel is not showing at face value.
        let mut notes: Vec<String> = Vec::new();
        // Columns this log simply does not carry are named, so an empty-looking
        // panel is never ambiguous between "not fitted" and "nothing happened".
        let missing: Vec<&str> = traces
            .iter()
            .chain(secondary.iter())
            .filter(|(_, t)| t.is_none())
            .map(|(l, _)| l.label)
            .collect();
        if !missing.is_empty() {
            notes.push(format!("no data: {}", missing.join(", ")));
        }
        for (range, unit) in [
            (Some(&axis), panel.unit),
            (
                s_axis.as_ref(),
                panel.secondary.as_ref().map_or("", |s| s.unit),
            ),
        ] {
            if let Some((lo, hi)) = range.and_then(|a| a.clipped) {
                notes.push(format!(
                    "axis excludes outliers — full range {lo:.1} to {hi:.1} {unit}"
                ));
            }
        }
        if !notes.is_empty() {
            let (w, _) = area.dim_in_pixel();
            area.draw_text(
                &notes.join("   ·   "),
                &TextStyle::from((theme::FONT, theme::F_NOTE).into_font())
                    .color(&theme::MUTED)
                    .pos(Pos::new(HPos::Right, VPos::Center)),
                (w as i32 - 40, 34),
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
        right_gutter: i32,
    ) -> Result<()> {
        let (w, h) = area.dim_in_pixel();

        // Still a real chart, even with nothing to put in it. The bottom panel
        // of a column owns that column's tick labels, and whether that panel
        // happens to have data is chance — on this log the bottom two cells are
        // payload columns the flight never wrote, and returning early here left
        // two entire columns of the auxiliary figure with no time axis at all.
        let mut chart = ChartBuilder::on(area)
            .caption(
                panel.title,
                TextStyle::from((theme::FONT, theme::F_CAPTION).into_font())
                    .color(&theme::MUTED),
            )
            .margin_right(if right_gutter > 0 { 14 } else { 40 })
            .margin_left(14)
            .margin_bottom(12)
            .x_label_area_size(if panel.x_labels { X_LABELS_H } else { 0 })
            .y_label_area_size(Y_GUTTER)
            .right_y_label_area_size(right_gutter)
            .build_cartesian_2d(self.x_range.0..self.x_range.1, 0f64..1f64)
            .plot()?;

        {
            let mut binding = chart.configure_mesh();
            let mesh = binding
                .disable_y_mesh()
                .light_line_style(theme::GRID.mix(0.55))
                .bold_line_style(theme::GRID)
                .axis_style(theme::AXIS)
                .label_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                )
                .y_labels(0)
                .axis_desc_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                );
            if panel.x_labels {
                mesh.x_desc("T+ seconds");
            } else {
                mesh.disable_x_axis();
            }
            mesh.draw().plot()?;
        }

        self.draw_stage_bands(&mut chart, 0.0, 1.0)?;
        self.draw_event_rules(&mut chart, 0.0, 1.0)?;
        self.draw_plot_border(&mut chart, 0.0, 1.0)?;

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
            &TextStyle::from((theme::FONT, theme::F_NOTE).into_font())
                .color(&theme::MUTED)
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
        x_labels: bool,
        right_gutter: i32,
    ) -> Result<()> {
        area.fill(&theme::PANEL_BG).plot()?;
        let n = lanes.len() as f64;

        let mut chart = ChartBuilder::on(area)
            .caption(
                title,
                TextStyle::from((theme::FONT, theme::F_CAPTION).into_font()).color(&theme::TEXT),
            )
            .margin_right(if right_gutter > 0 { 14 } else { 40 })
            .margin_left(14)
            .margin_bottom(12)
            .x_label_area_size(if x_labels { X_LABELS_H } else { 0 })
            .y_label_area_size(Y_GUTTER)
            .right_y_label_area_size(right_gutter)
            .build_cartesian_2d(self.x_range.0..self.x_range.1, 0f64..n)
            .plot()?;

        {
            let mut binding = chart.configure_mesh();
            let mesh = binding
                .disable_y_mesh()
                .light_line_style(theme::GRID.mix(0.55))
                .bold_line_style(theme::GRID)
                .axis_style(theme::AXIS)
                .label_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                )
                .y_labels(0)
                .axis_desc_style(
                    TextStyle::from((theme::FONT, theme::F_TICK).into_font())
                        .color(&theme::MUTED),
                );
            if x_labels {
                mesh.x_desc("T+ seconds");
            } else {
                mesh.disable_x_axis();
            }
            mesh.draw().plot()?;
        }

        self.draw_stage_bands(&mut chart, 0.0, n)?;
        self.draw_event_rules(&mut chart, 0.0, n)?;
        self.draw_plot_border(&mut chart, 0.0, n)?;

        let label_style = TextStyle::from((theme::FONT, theme::F_LANE).into_font())
            .color(&theme::TEXT)
            .pos(Pos::new(HPos::Right, VPos::Center));
        // Bound rather than inlined: `color` borrows, and this style outlives
        // the statement that builds it.
        let dim = theme::MUTED.mix(0.55);
        let absent_style = TextStyle::from((theme::FONT, theme::F_LANE).into_font())
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
        let label_right = px.start - base_x - 16;
        // A pulse a few rows long is well under a pixel wide at this scale.
        // Widening it to a floor keeps it visible; without this a pyro fire —
        // the shortest and most important event in the log — renders as nothing.
        let min_width = (self.x_range.1 - self.x_range.0) / plot_w.max(1.0) * 3.0;

        for (i, (label, column, color)) in lanes.iter().enumerate() {
            // Lane 0 at the top reads in the order the list is written.
            let top = n - i as f64;
            let bottom = top - 1.0;
            let inset = 0.24;

            let values = self.column(column);
            let y_px = plot_top + plot_h * (i as f64 + 0.5) / n;
            area.draw_text(
                label,
                if values.is_some() {
                    &label_style
                } else {
                    &absent_style
                },
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
                        theme::GRID.stroke_width(2),
                    )))
                    .plot()?;

                let spans = true_spans(
                    &self.times,
                    values,
                    self.window.0,
                    self.window.1,
                );
                chart
                    .draw_series(spans.iter().map(|&(a, b)| {
                        Rectangle::new(
                            [
                                (a, bottom + inset),
                                (b.max(a + min_width), top - inset),
                            ],
                            color.mix(0.9).filled(),
                        )
                    }))
                    .plot()?;
            }
        }
        Ok(())
    }

    /// Wash the flight-stage bands in behind a chart's data, then the burn on
    /// top of them.
    ///
    /// Two layers rather than one, because the burn is not a stage: it starts
    /// inside `Ascent` and ends inside it, and cutting `Ascent` into two bands
    /// would claim the flight computer changed state at burnout when it did
    /// not. Laid over, the reader sees one continuous `Ascent` with its powered
    /// portion picked out — which is what actually happened.
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
        // Under the burn, because the two do not overlap in practice (the
        // brakes cannot be permitted while the motor is lit) and where they do,
        // the burn is the more surprising fact.
        //
        // Widened to a pixel floor, for the same reason the pyro lanes are:
        // the gate is evaluated per sample and its first opening can be a
        // couple of dozen milliseconds before the filter's birth transient
        // shuts it again. That is a finding, and at 25 ms on a 45 s axis it is
        // a third of a pixel wide — drawn honestly, it would not be drawn.
        let min_width = {
            let (px, _) = chart.plotting_area().get_pixel_range();
            (self.x_range.1 - self.x_range.0) / (px.end - px.start).max(1) as f64 * 3.0
        };
        for &(a, b) in &self.brakes_spans {
            let (a, b) = (a.max(self.x_range.0), b.max(a + min_width).min(self.x_range.1));
            if b > a {
                chart
                    .draw_series(std::iter::once(Rectangle::new(
                        [(a, y_lo), (b, y_hi)],
                        theme::brakes_color().filled(),
                    )))
                    .plot()?;
            }
        }
        if let Some((a, b)) = self.burn_span {
            // Clipped to the axis: the airbrakes figure ends at apogee and the
            // others do not, but the burn is the same span of seconds on all
            // three, so the clip lives here rather than at each call site.
            let (a, b) = (a.max(self.x_range.0), b.min(self.x_range.1));
            if b > a {
                chart
                    .draw_series(std::iter::once(Rectangle::new(
                        [(a, y_lo), (b, y_hi)],
                        theme::burn_color().filled(),
                    )))
                    .plot()?;
            }
        }
        Ok(())
    }

    /// Outline the plotting area.
    ///
    /// Only the bottom panel of a column draws an x axis, which is the point of
    /// sharing the axis — but the axis line was also the only thing dividing one
    /// stacked panel from the next, and without it a noisy trace appears to
    /// spill into its neighbour. The border restores the division at no cost in
    /// height.
    fn draw_plot_border<DB: DrawingBackend>(
        &self,
        chart: &mut ChartContext<'_, DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
        y_lo: f64,
        y_hi: f64,
    ) -> Result<()>
    where
        DB::ErrorType: 'static,
    {
        chart
            .draw_series(std::iter::once(Rectangle::new(
                [(self.x_range.0, y_lo), (self.x_range.1, y_hi)],
                theme::AXIS.mix(0.55).stroke_width(2),
            )))
            .plot()?;
        Ok(())
    }

    /// Broken vertical rules at each event, on every panel.
    ///
    /// Drawn under the traces, not over them: the rule is there to locate a
    /// feature in the data, and a rule that hides the feature has inverted its
    /// own purpose.
    fn draw_event_rules<DB: DrawingBackend>(
        &self,
        chart: &mut ChartContext<'_, DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
        y_lo: f64,
        y_hi: f64,
    ) -> Result<()>
    where
        DB::ErrorType: 'static,
    {
        for rule in self.rules() {
            let (dash, gap, style) = if rule.dotted {
                (3, 13, rule.color.mix(0.85).stroke_width(3))
            } else {
                (5, 9, rule.color.mix(0.55).stroke_width(2))
            };
            chart
                .draw_series(DashedLineSeries::new(
                    vec![(rule.at_s, y_lo), (rule.at_s, y_hi)],
                    dash,
                    gap,
                    style,
                ))
                .plot()?;
        }
        Ok(())
    }

    /// Name the event rules, on the top panel of a column only.
    ///
    /// Once per column rather than once per panel: the rules are the same on
    /// every panel below, so repeating the names would be eleven copies of the
    /// same six words on the auxiliary figure.
    fn draw_event_labels<DB: DrawingBackend>(
        &self,
        chart: &mut ChartContext<'_, DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
        y_lo: f64,
        y_hi: f64,
        legend: Option<(f64, f64)>,
    ) -> Result<()>
    where
        DB::ErrorType: 'static,
    {
        let span = y_hi - y_lo;
        let x_span = self.x_range.1 - self.x_range.0;
        let (plot_w, plot_h) = {
            let (px, py) = chart.plotting_area().get_pixel_range();
            (
                (px.end - px.start).max(1) as f64,
                (py.end - py.start).max(1) as f64,
            )
        };
        // Labels are placed on the first level that is still clear at this x,
        // rather than alternating between two. Events cluster — liftoff,
        // burnout, apogee and drogue deployment can fall inside the first tenth
        // of the axis — and a fixed two-level scheme overstrikes as soon as
        // three of them are close.
        // Ten, because the levels are now shared between two demands: the
        // packing that keeps neighbouring labels apart, and the offset that
        // clears the legend. On the auxiliary figure's 500 s axis the whole
        // flight — five events — falls inside the legend's own width, so all
        // five stack below it, and a shorter ladder ran out and overstruck.
        // The step is tightened to match, so ten levels still fit in the top
        // three quarters of a panel.
        const LEVELS: usize = 10;
        const FIRST: f64 = 0.055;
        const STEP: f64 = 0.07;
        // The lowest level clear of the legend, and how far right the legend
        // reaches — a label whose text starts beyond that can use level 0
        // however tall the legend is.
        let (legend_right, blocked_levels) = match legend {
            Some((w, h)) => (
                self.x_range.0 + w / plot_w * x_span,
                (((h / plot_h) - FIRST) / STEP).ceil().max(0.0) as usize,
            ),
            None => (f64::NEG_INFINITY, 0),
        };
        let mut level_free = [f64::NEG_INFINITY; LEVELS];
        for rule in self.rules() {
            // ~0.52 em per character, converted from pixels into axis units so
            // the packing holds at any figure width.
            let text_w =
                rule.label.chars().count() as f64 * theme::F_EVENT as f64 * 0.52 / plot_w
                    * x_span;
            // An event in the last fifth of the axis would run its label off the
            // right edge, so it hangs to the left of its rule instead.
            let near_right = rule.at_s > self.x_range.0 + x_span * 0.8;
            let (anchor, dx) = if near_right {
                (HPos::Right, -x_span * 0.006)
            } else {
                (HPos::Left, x_span * 0.006)
            };
            let left = if near_right {
                rule.at_s + dx - text_w
            } else {
                rule.at_s + dx
            };
            let lowest = if left < legend_right {
                blocked_levels.min(LEVELS - 1)
            } else {
                0
            };
            let level = (lowest..LEVELS)
                .find(|&l| left >= level_free[l])
                .unwrap_or(LEVELS - 1);
            level_free[level] = left + text_w + x_span * 0.012;
            let y = y_hi - span * (FIRST + STEP * level as f64);
            chart
                .draw_series(std::iter::once(Text::new(
                    rule.label.clone(),
                    (rule.at_s + dx, y),
                    TextStyle::from((theme::FONT, theme::F_EVENT).into_font())
                        .color(&rule.color)
                        .pos(Pos::new(anchor, VPos::Center)),
                )))
                .plot()?;
        }
        Ok(())
    }
}
