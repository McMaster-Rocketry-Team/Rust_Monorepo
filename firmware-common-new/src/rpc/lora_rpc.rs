use core::mem::MaybeUninit;

use rkyv::{Archive, Deserialize, Serialize};

use crate::create_rpc;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Archive, Deserialize, Serialize)]
pub struct RpcPacketStatus {
    pub rssi: i16,
    pub snr: i16,
}

create_rpc! {
    lora
    0 configure | frequency: u32, sf: u8, bw: u32, cr: u8, power: i32| -> ()
    1 has_new_rx | a: u8 | -> (rx_count: u32)
    2 get_rx | | -> (data: [u8; 256], status: RpcPacketStatus)
    3 tx | data: [u8; 256] | -> ()
}
