use crate::vlp::lora_config::LoraConfig;
use core::mem::MaybeUninit;
// use rkyv::{Archive, Deserialize, Serialize};
use crate::vlp::packets::MAX_VLP_PACKET_SIZE;

use crate::create_rpc;

create_rpc! {
    gs
    enums {
        enum GSUplinkState {
            UplinkQueueEmpty,
            UplinkQueuing,
            UplinkErrorRadio,
            UplinkErrorAckNotReceived,
            UplinkErrorInvalidAck,
        }
    }
    0 init | config: LoraConfig, key: [u8; 32] | -> ()

    1 poll_uplink_state | | -> (state: GSUplinkState)
    2 send_uplink | packet: [u8; MAX_VLP_PACKET_SIZE], len: u32 | -> ()

    3 poll_downlink_state | | -> (has_downlink: bool)
    4 get_downlink | | -> (packet: [u8; MAX_VLP_PACKET_SIZE], len: u32)
}
