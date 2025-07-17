use anyhow::Result;
use base64::prelude::*;
use firmware_common_new::bootloader::generate_public_key;
use log::info;
use rand::Rng as _;

use crate::{args::GenOtaKeyCli, gs::config::GroundStationConfig};

pub fn gen_ota_key(args: GenOtaKeyCli) -> Result<()> {
    let secret_key = rand::rng().random::<[u8; 32]>();
    let public_key = generate_public_key(&secret_key);
    std::fs::write(args.secret_key_path, BASE64_STANDARD.encode(&secret_key))?;
    std::fs::write(args.public_key_path, BASE64_STANDARD.encode(&public_key))?;
    info!("keys generated");
    Ok(())
}

pub fn gen_vlp_key() -> Result<()> {
    let key = rand::rng().random::<[u8; 32]>();

    let mut gs_config = GroundStationConfig::load()?;
    gs_config.vlp_key = key;
    gs_config.save()?;

    info!("VLP key generated and saved to config: {:?}", &key);
    info!("Config path: {}", GroundStationConfig::get_config_path().display());

    Ok(())
}