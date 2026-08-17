//! Colours and type for the rendered figures.
//!
//! A dark ground is not decoration here. These charts are read as overlaid
//! traces — four or five signals sharing one panel — and saturated strokes hold
//! their identity against a dark background far better than against white,
//! where they have to be darkened toward each other to stay legible. It also
//! matches every other instrument someone looks at flight data on.

use plotters::prelude::*;

pub const BG: RGBColor = RGBColor(0x0B, 0x0D, 0x12);
pub const PANEL_BG: RGBColor = RGBColor(0x15, 0x19, 0x23);
pub const GRID: RGBColor = RGBColor(0x22, 0x26, 0x31);
pub const AXIS: RGBColor = RGBColor(0x39, 0x40, 0x51);
pub const TEXT: RGBColor = RGBColor(0xD7, 0xDC, 0xE8);
pub const MUTED: RGBColor = RGBColor(0x79, 0x83, 0x99);

/// Trace colours, ordered so that any prefix of the list stays distinguishable —
/// the first three carry most panels and are far apart in hue, and none of them
/// collide with the semantic colours below.
pub const CYAN: RGBColor = RGBColor(0x45, 0xC7, 0xE8);
pub const AMBER: RGBColor = RGBColor(0xF0, 0xA9, 0x3B);
pub const VIOLET: RGBColor = RGBColor(0xA9, 0x8B, 0xF5);
pub const GREEN: RGBColor = RGBColor(0x5F, 0xD3, 0x7E);
pub const CORAL: RGBColor = RGBColor(0xF0, 0x62, 0x5A);
pub const BLUE: RGBColor = RGBColor(0x6E, 0x9B, 0xF7);
pub const ROSE: RGBColor = RGBColor(0xF5, 0x71, 0xAC);

/// Reserved for things that mean "this went wrong" — never used as an ordinary
/// trace colour, so its appearance always carries the same meaning.
pub const ALERT: RGBColor = RGBColor(0xFF, 0x5C, 0x4D);

pub const FONT: &str = "sans-serif";

/// The hue a flight stage is identified by.
///
/// Split from [`stage_color`] because the two are used at opposite opacities: a
/// band behind live traces has to be almost invisible, while the key that names
/// it has to be readable. Deriving one from the other keeps them the same hue.
pub fn stage_hue(stage: u8) -> RGBColor {
    match stage {
        0 | 1 => RGBColor(0x6E, 0x78, 0x90), // LowPower / SelfTest
        2 => RGBColor(0x8A, 0x95, 0xAD),     // Armed
        3 => AMBER,                          // Ascent
        4 => CYAN,                           // DrogueChute
        5 => GREEN,                          // MainChute
        6 => RGBColor(0x6E, 0x78, 0x90),     // Landed
        _ => ALERT,                          // FailedToReachMinApogee
    }
}

/// Background wash for a flight stage. Kept faint: these are context for the
/// traces, and a band that competes with the data has failed at its job.
pub fn stage_color(stage: u8) -> RGBAColor {
    let opacity = match stage {
        6 => 0.22,     // Landed reads as "the flight is over", so it is heavier
        2 => 0.16,     // Armed
        0 | 1 => 0.16, // LowPower / SelfTest
        7 => 0.16,     // FailedToReachMinApogee
        _ => 0.12,     // the airborne stages, behind the densest traces
    };
    stage_hue(stage).mix(opacity)
}

pub fn stage_name(stage: u8) -> &'static str {
    match stage {
        0 => "LowPower",
        1 => "SelfTest",
        2 => "Armed",
        3 => "Ascent",
        4 => "Drogue",
        5 => "Main",
        6 => "Landed",
        7 => "FailedApogee",
        _ => "?",
    }
}
