pub mod log_saver;
pub mod target_log;

use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use anyhow::Result;
use cursive::{
    Printer, Rect, Vec2, View,
    direction::Direction,
    event::{Callback, Event, EventResult, MouseButton, MouseEvent},
    theme::{BaseColor, Color, ColorStyle, Effects, Palette, Style},
    utils::markup::StyledString,
    view::{CannotFocus, Nameable as _, Resizable, ScrollStrategy, Scrollable as _, scroll},
    views::{
        Button, Checkbox, Dialog, EditView, LinearLayout, ListView, NamedView, Panel, ScrollView,
        TextView,
    },
};
use log::Level;
use pad::PadStr;
use target_log::{DefmtLogInfo, TargetLog};
use tokio::{
    sync::{broadcast, watch},
    time,
};

#[derive(Debug, Clone, Copy)]
pub enum LogViewerStatus {
    Initialize,
    Normal,
    ChunkError,
    Overrun,
}


pub fn log_level_foreground_color(log_level: Level) -> Color {
    match log_level {
        Level::Trace => Color::Rgb(127, 127, 127),
        Level::Debug => Color::Rgb(0, 0, 255),
        Level::Info => Color::Rgb(0, 160, 0),
        Level::Warn => Color::Rgb(127, 127, 0),
        Level::Error => Color::Rgb(255, 0, 0),
    }
}
