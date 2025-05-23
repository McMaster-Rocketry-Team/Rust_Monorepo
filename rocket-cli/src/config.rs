use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Serialize, Deserialize)]
pub struct LogViewerConfig {
    pub levels: LevelFilters,
    pub devices: DeviceFilters,
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
    pub icarus: bool,
    pub payload_activation: bool,
    pub rocket_wifi: bool,
    pub ozys: bool,
    pub bulkhead: bool,
    pub eps1: bool,
    pub eps2: bool,
    pub aerorust: bool,
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
                icarus: true,
                payload_activation: true,
                rocket_wifi: true,
                ozys: true,
                bulkhead: true,
                eps1: true,
                eps2: true,
                aerorust: true,
            },
        }
    }
}

impl LogViewerConfig {
    pub fn exists() -> bool {
        let config_path = Self::get_config_path();
        config_path.exists()
    }

    pub fn load() -> Result<Self> {
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
}
