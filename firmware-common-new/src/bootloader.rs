use salty::{PublicKey, Signature};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum VerifyFirmwareError {
    SaltyError(#[cfg_attr(feature = "defmt", defmt(Debug2Format))] salty::Error),
}

pub fn generate_public_key(secret: &[u8; 32]) -> [u8; 32] {
    let keypair: salty::Keypair = salty::Keypair::from(secret);

    keypair.public.to_bytes()
}

pub fn sign_firmware(firmware_sha512: &[u8; 64], secret: &[u8; 32]) -> [u8; 64] {
    let keypair: salty::Keypair = salty::Keypair::from(secret);
    let signature = keypair.sign_prehashed(firmware_sha512, None);
    signature.to_bytes()
}

pub fn verify_firmware(
    firmware_sha512: &[u8; 64],
    signature: &[u8; 64],
    public_key: &[u8; 32],
) -> Result<(), VerifyFirmwareError> {
    let signature = Signature::try_from(signature).unwrap();

    let public_key = PublicKey::try_from(public_key).map_err(VerifyFirmwareError::SaltyError)?;
    public_key
        .verify_prehashed(firmware_sha512, &signature, None)
        .map_err(VerifyFirmwareError::SaltyError)?;
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn firmware_sign_verify() {
        let firmware_sha512 = [69u8; 64];
        let secret = [42u8; 32];
        let public_key = generate_public_key(&secret);

        let signature = sign_firmware(&firmware_sha512, &secret);

        verify_firmware(&firmware_sha512, &signature, &public_key).unwrap();
        verify_firmware(&[68u8; 64], &signature, &public_key).unwrap_err();
    }
}
