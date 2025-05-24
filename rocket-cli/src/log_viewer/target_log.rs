use cursive::theme::Color;
use firmware_common_new::can_bus::node_types::*;
use log::Level;

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum NodeTypeEnum {
    VoidLake,
    AMP,
    AMPSpeedBridge,
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
            AMP_SPEED_BRIDGE_NODE_TYPE => Self::AMPSpeedBridge,
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
            NodeTypeEnum::AMPSpeedBridge => AMP_SPEED_BRIDGE_NODE_TYPE,
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
            NodeTypeEnum::AMPSpeedBridge => "ASB",
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
            NodeTypeEnum::VoidLake => Color::Rgb(194, 238, 218),
            NodeTypeEnum::AMP => Color::Rgb(215, 215, 183),
            NodeTypeEnum::AMPSpeedBridge => Color::Rgb(192, 248, 240),
            NodeTypeEnum::ICARUS => Color::Rgb(212, 210, 240),
            NodeTypeEnum::PayloadActivation => Color::Rgb(248, 219, 194),
            NodeTypeEnum::RocketWifi => Color::Rgb(233, 242, 234),
            NodeTypeEnum::OZYS => Color::Rgb(244, 230, 197),
            NodeTypeEnum::Bulkhead => Color::Rgb(206, 234, 255),
            NodeTypeEnum::EPS1 => Color::Rgb(228, 235, 227),
            NodeTypeEnum::EPS2 => Color::Rgb(202, 217, 202),
            NodeTypeEnum::AeroRust => Color::Rgb(252, 218, 246),
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
    pub location: Option<DefmtLocationInfo>,
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
