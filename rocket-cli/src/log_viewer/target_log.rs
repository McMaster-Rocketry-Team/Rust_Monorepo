use cursive::theme::Color;
use firmware_common_new::can_bus::node_types::*;
use log::Level;

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq, Hash)]
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

impl From<u8> for NodeTypeEnum {
    fn from(value: u8) -> Self {
        match value {
            VOID_LAKE_NODE_TYPE => Self::VoidLake,
            AMP_NODE_TYPE => Self::AMP,
            ICARUS_NODE_TYPE => Self::ICARUS,
            PAYLOAD_ACTIVATION_NODE_TYPE => Self::PayloadActivation,
            PAYLOAD_ROCKET_WIFI_NODE_TYPE => Self::RocketWifi,
            OZYS_NODE_TYPE => Self::OZYS,
            BULKHEAD_NODE_TYPE => Self::Bulkhead,
            PAYLOAD_EPS1_NODE_TYPE => Self::EPS1,
            PAYLOAD_EPS2_NODE_TYPE => Self::EPS2,
            AERO_RUST_NODE_TYPE => Self::AeroRust,
            _ => Self::Other,
        }
    }
}

impl Into<u8> for NodeTypeEnum {
    fn into(self) -> u8 {
        match self {
            NodeTypeEnum::VoidLake => VOID_LAKE_NODE_TYPE,
            NodeTypeEnum::AMP => AMP_NODE_TYPE,
            NodeTypeEnum::ICARUS => ICARUS_NODE_TYPE,
            NodeTypeEnum::PayloadActivation => PAYLOAD_ACTIVATION_NODE_TYPE,
            NodeTypeEnum::RocketWifi => PAYLOAD_ROCKET_WIFI_NODE_TYPE,
            NodeTypeEnum::OZYS => OZYS_NODE_TYPE,
            NodeTypeEnum::Bulkhead => BULKHEAD_NODE_TYPE,
            NodeTypeEnum::EPS1 => PAYLOAD_EPS1_NODE_TYPE,
            NodeTypeEnum::EPS2 => PAYLOAD_EPS2_NODE_TYPE,
            NodeTypeEnum::AeroRust => AERO_RUST_NODE_TYPE,
            NodeTypeEnum::Other => unimplemented!(),
        }
    }
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
            NodeTypeEnum::Other => "??",
        }
    }

    pub fn background_color(&self) -> Color {
        match self {
            NodeTypeEnum::VoidLake => Color::Rgb(205, 232, 255),
            NodeTypeEnum::AMP => Color::Rgb(255, 254, 233),
            NodeTypeEnum::ICARUS => Color::Rgb(255, 227, 207),
            NodeTypeEnum::PayloadActivation => Color::Rgb(207, 248, 255),
            NodeTypeEnum::RocketWifi => Color::Rgb(245, 219, 239),
            NodeTypeEnum::OZYS => Color::Rgb(232, 255, 231),
            NodeTypeEnum::Bulkhead => Color::Rgb(229, 237, 255),
            NodeTypeEnum::EPS1 => Color::Rgb(216, 255, 244),
            NodeTypeEnum::EPS2 => Color::Rgb(204, 232, 238),
            NodeTypeEnum::AeroRust => Color::Rgb(227, 242, 240),
            NodeTypeEnum::Other => Color::Rgb(255, 255, 255),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DefmtLocationInfo {
    pub module_path: String,
    pub file_path: String,
    pub line_number: String,
}

#[derive(Debug, Clone)]
pub struct DefmtLogInfo {
    pub log_level: Level,
    pub timestamp: Option<f64>,
    pub location: Option<DefmtLocationInfo>
}

#[derive(Debug, Clone)]
pub struct TargetLog {
    pub node_type: NodeTypeEnum,
    pub node_id: Option<u16>,
    pub log_content: String,

    pub defmt: Option<DefmtLogInfo>,
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
        Level::Debug => Color::Rgb(0, 0, 255),
        Level::Info => Color::Rgb(0, 160, 0),
        Level::Warn => Color::Rgb(127, 127, 0),
        Level::Error => Color::Rgb(255, 0, 0),
    }
}
