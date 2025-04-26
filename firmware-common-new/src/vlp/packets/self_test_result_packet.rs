pub struct SelfTestResultPacket {
    imu_ok: bool,
    baro_ok: bool,
    mag_ok: bool,
    gps_ok: bool,
    sd_ok: bool,
    can_bus_ok: bool,

    amp_reachable: bool,
    amp_out1_ok: bool,
    amp_out2_ok: bool,
    amp_out3_ok: bool,
    amp_out4_ok: bool,

    icarus_reachable: bool,
    icarus_imu_ok: bool,
    icarus_baro_ok: bool,
    icarus_servo_ok: bool,

    ozys1_reachable: bool,
    ozys1_sd_ok: bool,

    ozys2_reachable: bool,
    ozys2_sd_ok: bool,

    payload_activation_reachable: bool,

    payload_eps1_reachable: bool,
    payload_eps1_battery1_ok: bool,
    payload_eps1_battery2_ok: bool,
    payload_eps1_out_3v3_ok: bool,
    payload_eps1_out_5v_ok: bool,
    payload_eps1_out_9v_ok: bool,

    payload_eps2_reachable: bool,
    payload_eps2_battery1_ok: bool,
    payload_eps2_battery2_ok: bool,
    payload_eps2_out_3v3_ok: bool,
    payload_eps2_out_5v_ok: bool,
    payload_eps2_out_9v_ok: bool,

    aero_rust_reachable: bool,
}