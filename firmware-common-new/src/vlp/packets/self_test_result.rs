use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
pub struct SelfTestResultPacket {
    pub imu_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub sd_ok: bool,
    pub can_bus_ok: bool,

    pub amp_reachable: bool,
    pub amp_out1_ok: bool,
    pub amp_out2_ok: bool,
    pub amp_out3_ok: bool,
    pub amp_out4_ok: bool,

    pub icarus_reachable: bool,
    pub icarus_imu_ok: bool,
    pub icarus_baro_ok: bool,
    pub icarus_servo_ok: bool,

    pub ozys1_reachable: bool,
    pub ozys1_sd_ok: bool,

    pub ozys2_reachable: bool,
    pub ozys2_sd_ok: bool,

    pub aero_rust_reachable: bool,

    pub payload_activation_pcb_reachable: bool,

    pub rocket_wifi_reachable: bool,

    pub payload_eps1_reachable: bool,
    pub payload_eps1_battery1_ok: bool,
    pub payload_eps1_battery2_ok: bool,
    pub payload_eps1_out_3v3_ok: bool,
    pub payload_eps1_out_5v_ok: bool,
    pub payload_eps1_out_9v_ok: bool,

    pub payload_eps2_reachable: bool,
    pub payload_eps2_battery1_ok: bool,
    pub payload_eps2_battery2_ok: bool,
    pub payload_eps2_out_3v3_ok: bool,
    pub payload_eps2_out_5v_ok: bool,
    pub payload_eps2_out_9v_ok: bool,
}
