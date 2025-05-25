use anyhow::Result;
use log::Level;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

use crate::args::NodeTypeEnum;

use super::log::target_log::TargetLog;


#[derive(Debug, Serialize, Deserialize)]
pub struct LogViewerConfig {
    pub levels: LevelFilters,
    pub devices: DeviceFilters,
    pub module: String,
    pub search: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LevelFilters {
    pub trace: bool,
    pub debug: bool,
    pub info: bool,
    pub warn: bool,
    pub error: bool,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct DeviceFilters {
    pub void_lake: bool,
    pub amp: bool,
    pub amp_speed_bridge: bool,
    pub icarus: bool,
    pub payload_activation: bool,
    pub rocket_wifi: bool,
    pub ozys: bool,
    pub bulkhead: bool,
    pub eps1: bool,
    pub eps2: bool,
    pub aerorust: bool,
    pub other: bool,
}

impl Default for LogViewerConfig {
    fn default() -> Self {
        Self {
            levels: LevelFilters {
                trace: true,
                debug: true,
                info: true,
                warn: true,
                error: true,
            },
            devices: DeviceFilters {
                void_lake: true,
                amp: true,
                amp_speed_bridge: true,
                icarus: true,
                payload_activation: true,
                rocket_wifi: true,
                ozys: true,
                bulkhead: true,
                eps1: true,
                eps2: true,
                aerorust: true,
                other: true,
            },
            module: String::new(),
            search: String::new(),
        }
    }
}

impl LogViewerConfig {
    pub fn exists() -> bool {
        let config_path = Self::get_config_path();
        config_path.exists()
    }

    pub fn load() -> Result<Self> {
        let config = Self::try_load();
        if config.is_ok() {
            return config;
        }

        let config = Self::default();
        config.save()?;
        return Ok(config);
    }

    pub fn try_load() -> Result<Self> {
        let config_path = Self::get_config_path();

        if !config_path.exists() {
            let config = Self::default();
            config.save()?;
            return Ok(config);
        }

        let config_str = fs::read_to_string(config_path)?;
        let config = toml::from_str(&config_str)?;
        Ok(config)
    }

    pub fn save(&self) -> Result<()> {
        let config_path = Self::get_config_path();
        let config_str = toml::to_string_pretty(self)?;

        if let Some(parent) = config_path.parent() {
            fs::create_dir_all(parent)?;
        }

        fs::write(config_path, config_str)?;
        Ok(())
    }

    fn get_config_path() -> PathBuf {
        ".rocket-cli.toml".into()
    }

    pub fn matches(&self, log: &TargetLog) -> bool {
        if let Some(defmt_info) = &log.defmt {
            let module_matches = if let Some(location) = &defmt_info.location {
                location.module_path.starts_with(&self.module)
            } else {
                self.module.is_empty()
            };
            let module_matches = defmt_info.log_level == Level::Error
                || defmt_info.log_level == Level::Warn
                || module_matches;

            let level_matches = match defmt_info.log_level {
                Level::Trace => self.levels.trace,
                Level::Debug => self.levels.debug,
                Level::Info => self.levels.info,
                Level::Warn => self.levels.warn,
                Level::Error => self.levels.error,
            };

            if !module_matches || !level_matches {
                return false;
            }
        }

        let device_matches = match log.node_type {
            NodeTypeEnum::VoidLake => self.devices.void_lake,
            NodeTypeEnum::AMP => self.devices.amp,
            NodeTypeEnum::AMPSpeedBridge => self.devices.amp_speed_bridge,
            NodeTypeEnum::ICARUS => self.devices.icarus,
            NodeTypeEnum::PayloadActivation => self.devices.payload_activation,
            NodeTypeEnum::RocketWifi => self.devices.rocket_wifi,
            NodeTypeEnum::OZYS => self.devices.ozys,
            NodeTypeEnum::Bulkhead => self.devices.bulkhead,
            NodeTypeEnum::EPS1 => self.devices.eps1,
            NodeTypeEnum::EPS2 => self.devices.eps2,
            NodeTypeEnum::AeroRust => self.devices.aerorust,
            NodeTypeEnum::Other => self.devices.other,
        };

        device_matches
    }
}
