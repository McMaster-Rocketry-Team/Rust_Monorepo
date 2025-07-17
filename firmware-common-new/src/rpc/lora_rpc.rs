use crate::vlp::lora_config::LoraConfig;
use core::mem::MaybeUninit;
use rkyv::{Archive, Deserialize, Serialize};

use crate::create_rpc;


#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Archive, Deserialize, Serialize)]
pub enum LoraRpcRxResult {
    Timeout,
    Error,
    Success {
        len: u8,
        data: [u8; 256],
        rssi: i16,
        snr: i16,
    }
}

create_rpc! {
    lora
    0 configure | config: LoraConfig | -> ()
    1 rx | timeout_ms: u32 | -> (result: LoraRpcRxResult)
    2 tx | len: u32, data: [u8; 256] | -> (success: bool)
}
