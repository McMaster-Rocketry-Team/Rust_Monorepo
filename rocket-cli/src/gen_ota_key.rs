use anyhow::Result;
use firmware_common_new::bootloader::generate_public_key;
use log::info;
use rand::Rng as _;

use crate::args::GenOtaKeyCli;

pub fn gen_ota_key(args: GenOtaKeyCli) -> Result<()> {
    let secret_key = rand::rng().random::<[u8; 32]>();
    let public_key = generate_public_key(&secret_key);
    std::fs::write(args.secret_key_path, &secret_key)?;
    std::fs::write(args.public_key_path, &public_key)?;
    info!("keys generated");
    Ok(())
}
