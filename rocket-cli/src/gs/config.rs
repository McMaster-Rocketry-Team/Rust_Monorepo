use std::{env, fs, path::PathBuf};

use anyhow::Result;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct GroundStationConfig {
    pub vlp_key: [u8; 32],
    pub frequency: u32,
    pub power: i32,
}

impl Default for GroundStationConfig {
    fn default() -> Self {
        Self {
            vlp_key: [42u8; 32],
            frequency: 915_100_000,
            power: 22,
        }
    }
}

impl GroundStationConfig {
    pub fn load() -> Result<Self> {
        let config = Self::try_load();
        if config.is_ok() {
            return config;
        }

        let config = Self::default();
        config.save()?;
        Ok(config)
    }

    pub fn try_load() -> Result<Self> {
        let Some(config_path) = Self::resolve_existing_config_path() else {
            return Ok(Self::default());
        };
        let config_str = fs::read_to_string(config_path)?;
        let config = toml::from_str(&config_str)?;
        Ok(config)
    }
    pub fn save(&self) -> Result<()> {
        let config_path = Self::get_config_path();
        let config_str = toml::to_string_pretty(self)?;
        if let Some(parent) = config_path.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        fs::write(config_path, config_str)?;
        Ok(())
    }
    /// Where new configs are written (portable).
    pub fn get_config_path() -> PathBuf {
        if let Some(path) = Self::env_config_path() {
            return path;
        }
        Self::local_config_path()
    }
    fn resolve_existing_config_path() -> Option<PathBuf> {
        Self::config_search_paths()
            .into_iter()
            .find(|path| path.exists())
    }
    fn config_search_paths() -> Vec<PathBuf> {
        let mut paths = Vec::with_capacity(3);
        if let Some(path) = Self::env_config_path() {
            paths.push(path);
        }
        paths.push(Self::local_config_path());
        paths.push(Self::os_config_path());
        paths
    }
    fn env_config_path() -> Option<PathBuf> {
        env::var_os("ROCKET_CLI_GS_CONFIG").map(PathBuf::from)
    }
    fn local_config_path() -> PathBuf {
        PathBuf::from("ground-station.toml")
    }
    fn os_config_path() -> PathBuf {
        directories::ProjectDirs::from("ca.macrocketry", "MacRocketry", "rocket-cli")
            .unwrap()
            .config_dir()
            .join("ground-station.toml")
    }
}
