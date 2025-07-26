use firmware_common_new::vlp::{
    client::VLPTXError,
    packets::{VLPDownlinkPacket, VLPUplinkPacket},
};
use lora_phy::mod_params::PacketStatus;

pub trait VLPClientTrait: Sync {
    fn send_nb(&self, packet: VLPUplinkPacket);
    fn try_get_send_result(&self) -> Option<Result<PacketStatus, VLPTXError>>;
    fn try_receive(&self) -> Option<(VLPDownlinkPacket, PacketStatus)>;
}

