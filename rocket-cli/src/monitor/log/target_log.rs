use cursive::theme::Color;
use log::Level;
use serde::{Deserialize, Serialize};

use crate::args::NodeTypeEnum;

impl NodeTypeEnum {
    pub fn short_name(&self) -> &'static str {
        match self {
            NodeTypeEnum::VoidLake => "VL",
            NodeTypeEnum::AMP => "AMP",
            NodeTypeEnum::ICARUS => "ICA",
            NodeTypeEnum::PayloadSDRM => "SDRM",
            NodeTypeEnum::OZYS => "OZY",
            NodeTypeEnum::Bulkhead => "BKH",
            NodeTypeEnum::AeroRust => "AR",
            NodeTypeEnum::Other => "??",
        }
    }

    pub fn background_color(&self) -> Color {
        match self {
            NodeTypeEnum::VoidLake => Color::Rgb(224, 246, 236),
            NodeTypeEnum::AMP => Color::Rgb(235, 235, 219),
            NodeTypeEnum::ICARUS => Color::Rgb(234, 232, 248),
            NodeTypeEnum::PayloadSDRM => Color::Rgb(252, 237, 224),
            NodeTypeEnum::OZYS => Color::Rgb(250, 242, 226),
            NodeTypeEnum::Bulkhead => Color::Rgb(230, 244, 255),
            NodeTypeEnum::AeroRust => Color::Rgb(254, 236, 250),
            NodeTypeEnum::Other => Color::Rgb(255, 255, 255),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefmtLocationInfo {
    pub module_path: String,
    pub file_path: String,
    pub line_number: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefmtLogInfo {
    pub log_level: Level,
    pub timestamp: Option<f64>,
    pub location: Option<DefmtLocationInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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