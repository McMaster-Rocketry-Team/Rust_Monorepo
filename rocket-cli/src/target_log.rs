use cursive::theme::Color;
use log::Level;

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
pub enum NodeTypeEnum {
    VoidLake,
    AMP,
    ICARUS,
    PayloadActivation,
    RocketWifi,
    OZYS,
    Bulkhead,
    EPS1,
    EPS2,
    AeroRust,
    Other,
}

impl NodeTypeEnum {
    pub fn short_name(&self) -> &'static str {
        match self {
            NodeTypeEnum::VoidLake => "VL",
            NodeTypeEnum::AMP => "AMP",
            NodeTypeEnum::ICARUS => "ICA",
            NodeTypeEnum::PayloadActivation => "PA",
            NodeTypeEnum::RocketWifi => "RW",
            NodeTypeEnum::OZYS => "OZY",
            NodeTypeEnum::Bulkhead => "BKH",
            NodeTypeEnum::EPS1 => "EP1",
            NodeTypeEnum::EPS2 => "EP2",
            NodeTypeEnum::AeroRust => "AR",
            NodeTypeEnum::Other => "??"
        }
    }

    pub fn background_color(&self) -> Color {
        match self {
            NodeTypeEnum::VoidLake => Color::Rgb(255, 205, 245),
            NodeTypeEnum::AMP => Color::Rgb(134, 219, 170),
            NodeTypeEnum::ICARUS => Color::Rgb(219, 255, 183),
            NodeTypeEnum::PayloadActivation => Color::Rgb(142, 174, 213),
            NodeTypeEnum::RocketWifi => Color::Rgb(227, 211, 135),
            NodeTypeEnum::OZYS => Color::Rgb(199, 212, 255),
            NodeTypeEnum::Bulkhead => Color::Rgb(255, 203, 144),
            NodeTypeEnum::EPS1 => Color::Rgb(117, 255, 249),
            NodeTypeEnum::EPS2 => Color::Rgb(181, 195, 168),
            NodeTypeEnum::AeroRust => Color::Rgb(172, 255, 230),
            NodeTypeEnum::Other => Color::Rgb(255, 255, 255)
        }
    }
}

#[derive(Debug, Clone)]
pub struct TargetLog {
    pub node_type: NodeTypeEnum,
    pub node_id: Option<u16>,
    pub log_content: String,
    pub crate_name: String,
    pub file_name: String,
    pub file_path: String,
    pub line_number: String,
    pub log_level: Level,
    pub module_path: String,
    pub timestamp: Option<f64>,
}

pub fn parse_log_level(s: &str) -> Level {
    match s.to_uppercase().as_str() {
        "DEBUG" => Level::Debug,
        "INFO" => Level::Info,
        "WARN" => Level::Warn,
        "ERROR" => Level::Error,
        _ => Level::Info,
    }
}

pub fn log_level_foreground_color(log_level: Level) -> Color {
    match log_level {
        Level::Trace => Color::Rgb(127, 127, 127),
        Level::Debug => Color::Rgb(0, 0, 0),
        Level::Info => Color::Rgb(0, 0, 255),
        Level::Warn => Color::Rgb(127, 127, 0),
        Level::Error => Color::Rgb(255, 0, 0),
    }
}
