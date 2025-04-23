use crate::can_bus::messages::node_status::NodeHealth;

pub struct SelfTestResultPacket {
    imu_ok: bool,
    baro_ok: bool,
    mag_ok: bool,
    gps_ok: bool,

    main_bulkhead_health: NodeHealth,
    drogue_bulkhead_health: NodeHealth,

    amp_out1_ok: bool,
    amp_out2_ok: bool,
    amp_out3_ok: bool,
    amp_out4_ok: bool,

    icarus_imu_ok: bool,
    icarus_baro_ok: bool,
    icarus_servo_ok: bool,

    ozys1_health: NodeHealth,
    ozys2_health: NodeHealth,

}