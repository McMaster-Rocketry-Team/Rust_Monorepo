//! Colours and type for the rendered figures.
//!
//! The figures are white-ground, which is what a chart pasted into a report or
//! printed for a debrief needs. That constrains the traces: the bright strokes
//! that read well on a dark ground wash out on white, so every trace colour here
//! is chosen at a lightness that holds against white, and they are spaced around
//! the hue circle rather than being tints of one accent — four or five of them
//! share a panel and have to stay tellable apart.

use plotters::prelude::*;

pub const BG: RGBColor = RGBColor(0xFF, 0xFF, 0xFF);
pub const PANEL_BG: RGBColor = RGBColor(0xFF, 0xFF, 0xFF);
pub const GRID: RGBColor = RGBColor(0xDF, 0xE3, 0xE8);
pub const AXIS: RGBColor = RGBColor(0x8C, 0x96, 0xA3);
pub const TEXT: RGBColor = RGBColor(0x16, 0x1A, 0x20);
pub const MUTED: RGBColor = RGBColor(0x62, 0x6C, 0x7A);

/// Trace colours, ordered so that any prefix of the list stays distinguishable —
/// the first three carry most panels and are far apart in hue.
pub const CYAN: RGBColor = RGBColor(0x0B, 0x6E, 0x8F);
pub const AMBER: RGBColor = RGBColor(0xB0, 0x6A, 0x00);
pub const VIOLET: RGBColor = RGBColor(0x63, 0x3B, 0xB0);
pub const GREEN: RGBColor = RGBColor(0x1F, 0x7A, 0x34);
pub const CORAL: RGBColor = RGBColor(0xC0, 0x27, 0x24);
pub const BLUE: RGBColor = RGBColor(0x1C, 0x54, 0xB8);
pub const ROSE: RGBColor = RGBColor(0xA3, 0x12, 0x59);

/// Reserved for things that mean "this went wrong" — never used as an ordinary
/// trace colour, so its appearance always carries the same meaning.
pub const ALERT: RGBColor = RGBColor(0xC8, 0x1E, 0x1E);

/// Event rules and their labels. Deliberately near-black rather than coloured:
/// they cross every panel, and a coloured rule would read as another trace.
pub const EVENT: RGBColor = RGBColor(0x3A, 0x42, 0x50);

pub const FONT: &str = "sans-serif";

// Type sizes for a 3840x2160 figure. Set explicitly rather than scaled from a
// 1080p base — the ratios that work at 1080p give type that is technically
// legible but small at 4K, so these are all a little more than double.
pub const F_TITLE: i32 = 58;
pub const F_SUBTITLE: i32 = 30;
pub const F_CAPTION: i32 = 38;
pub const F_TICK: i32 = 26;
pub const F_LEGEND: i32 = 27;
pub const F_LANE: i32 = 26;
pub const F_NOTE: i32 = 24;
pub const F_EVENT: i32 = 25;

/// The hue a flight stage is identified by.
///
/// Split from [`stage_color`] because the two are used at opposite opacities: a
/// band behind live traces has to be almost invisible, while the key that names
/// it has to be readable. Deriving one from the other keeps them the same hue.
pub fn stage_hue(stage: u8) -> RGBColor {
    match stage {
        0 | 1 => RGBColor(0x77, 0x82, 0x93), // LowPower / SelfTest
        2 => RGBColor(0x8D, 0x98, 0xA8),     // Armed
        3 => AMBER,                          // Ascent
        // Violet, not a tint of Ascent's amber: it is the stretch the KF is
        // frozen, which is why the deployment traces are blank across it, and
        // that has to be findable at a glance rather than inferred.
        MACH_LOCKOUT => VIOLET,
        4 => CYAN,                           // DrogueChute
        5 => GREEN,                          // MainChute
        6 => RGBColor(0x77, 0x82, 0x93),     // Landed
        _ => ALERT,                          // FailedToReachMinApogee
    }
}

/// Background wash for a flight stage. Kept faint: these are context for the
/// traces, and a band that competes with the data has failed at its job. On a
/// white ground the same alpha reads stronger than it did on a dark one, so
/// these are lower than the equivalent dark-theme values would be.
pub fn stage_color(stage: u8) -> RGBAColor {
    let opacity = match stage {
        6 => 0.14,               // Landed reads as "the flight is over"
        MACH_LOCKOUT => 0.10,    // it explains a blank panel; it has to be seen
        0 | 1 | 2 => 0.11,  // LowPower / SelfTest / Armed
        7 => 0.11,          // FailedToReachMinApogee
        _ => 0.075,         // the airborne stages, behind the densest traces
    };
    stage_hue(stage).mix(opacity)
}

/// The motor burn — ignition to burnout.
///
/// Not a flight stage: the flight computer has one `Ascent`, and the moment
/// the motor stops is invisible in it even though it is the moment the
/// airframe stops being driven and the air brakes become worth anything. So
/// the burn gets its own wash, laid over the stage band rather than replacing
/// it, and a hue no stage uses so the overlap cannot be mistaken for one.
pub const BURN_HUE: RGBColor = RGBColor(0xC0, 0x27, 0x24);

/// Background wash for the burn. Fainter than the stage bands it sits on top
/// of, because it is the second layer and the two add.
pub fn burn_color() -> RGBAColor {
    BURN_HUE.mix(0.09)
}

/// The stretch the MPC was permitted to open the brakes.
///
/// Like the burn, not a flight stage — it opens and closes on a gate the
/// software evaluates every sample, and can do so more than once inside one
/// `Ascent`.
pub const BRAKES_HUE: RGBColor = RGBColor(0x00, 0x7A, 0xB8);

/// Background wash for the brakes-permitted spans.
///
/// The lightest of the three washes despite being the one laid over an
/// already-tinted stage band, which is why it is a light tint of the hue
/// rather than the hue at a lower alpha: dropping the alpha instead let the
/// amber underneath dominate and the band read grey.
pub fn brakes_color() -> RGBAColor {
    RGBColor(0x62, 0xB6, 0xE0).mix(0.16)
}

/// The Mach lockout, as a stage code.
///
/// `FlightStage` is three bits on the wire with all eight codes spent, so the
/// firmware folds the deployment estimator's `MachLockout` into `Ascent` on
/// its way into the log — the variant is not recorded and has to be
/// reconstructed. 8 is the code it gets back here: outside the range the
/// format can ever carry, so it cannot collide with a stage a future log
/// actually stores.
pub const MACH_LOCKOUT: u8 = 8;

pub fn stage_name(stage: u8) -> &'static str {
    match stage {
        MACH_LOCKOUT => "Mach lockout",
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
