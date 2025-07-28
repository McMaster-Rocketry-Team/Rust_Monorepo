use anyhow::Result;
use base64::prelude::*;
use firmware_common_new::bootloader::generate_public_key;
use log::info;
use rand::Rng as _;

use crate::{args::{GenOtaKeyCli, GenVlpKeyCli}, gs::config::GroundStationConfig};

pub fn gen_ota_key(args: GenOtaKeyCli) -> Result<()> {
    let secret_key = rand::rng().random::<[u8; 32]>();
    let public_key = generate_public_key(&secret_key);

    let mut secret_key_path = args.key_directory.clone();
    secret_key_path.push("./secret.key");
    std::fs::write(&secret_key_path, BASE64_STANDARD.encode(&secret_key))?;

    let mut public_key_path = args.key_directory.clone();
    public_key_path.push("./public.key");
    std::fs::write(&public_key_path, BASE64_STANDARD.encode(&public_key))?;

    info!("Keys generated");
    info!("Secret key path: {}", secret_key_path.canonicalize().unwrap().display());
    info!("Public key path: {}", public_key_path.canonicalize().unwrap().display());

    Ok(())
}

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