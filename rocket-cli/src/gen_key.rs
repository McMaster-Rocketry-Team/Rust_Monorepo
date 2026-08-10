use anyhow::Result;
use base64::prelude::*;
use log::info;
use rand::Rng as _;

use crate::{args::GenVlpKeyCli, gs::config::GroundStationConfig};

pub fn gen_vlp_key(args: GenVlpKeyCli) -> Result<()> {
    let key = rand::rng().random::<[u8; 32]>();
    info!("VLP key generated");

    let mut gs_config = GroundStationConfig::load()?;
    gs_config.vlp_key = key;
    gs_config.save()?;

    info!("Saved as toml for rocket-cli: {}", &GroundStationConfig::get_config_path().canonicalize().unwrap().display());

    std::fs::write(&args.key_path, BASE64_STANDARD.encode(&key))?;
    info!("Saved as base64 for firmware: {}", &args.key_path.canonicalize().unwrap().display());

    Ok(())
}