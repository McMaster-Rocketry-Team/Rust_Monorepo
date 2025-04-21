#![no_std]

pub use firmware_common_new::can_bus::messages::payload_status::{
    EPSOutputStatus, EPSOutputStatusEnum, EPSStatus,
};
use firmware_common_new::can_bus::messages::{CanBusMessage, PayloadStatusMessage};

#[no_mangle]
pub extern "C" fn create_payload_status_message(
    buffer: *mut u8,
    buffer_length: usize,
    eps1_battery1_mv: u16,
    eps1_battery2_mv: u16,
    eps1_output_3v3_current_ma: u16,
    eps1_output_3v3_status: EPSOutputStatusEnum,
    eps1_output_5v_current_ma: u16,
    eps1_output_5v_status: EPSOutputStatusEnum,
    eps1_output_9v_current_ma: u16,
    eps1_output_9v_status: EPSOutputStatusEnum,
    eps2_battery1_mv: u16,
    eps2_battery2_mv: u16,
    eps2_output_3v3_current_ma: u16,
    eps2_output_3v3_status: EPSOutputStatusEnum,
    eps2_output_5v_current_ma: u16,
    eps2_output_5v_status: EPSOutputStatusEnum,
    eps2_output_9v_current_ma: u16,
    eps2_output_9v_status: EPSOutputStatusEnum,
    eps1_node_id: u16,
    eps2_node_id: u16,
    payload_esp_node_id: u16,
) -> usize {
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };

    let message = PayloadStatusMessage::new(
        EPSStatus {
            battery1_mv: eps1_battery1_mv,
            battery2_mv: eps1_battery2_mv,
            output_3v3: EPSOutputStatus {
                current_ma: eps1_output_3v3_current_ma.into(),
                status: eps1_output_3v3_status,
            },
            output_5v: EPSOutputStatus {
                current_ma: eps1_output_5v_current_ma.into(),
                status: eps1_output_5v_status,
            },
            output_9v: EPSOutputStatus {
                current_ma: eps1_output_9v_current_ma.into(),
                status: eps1_output_9v_status,
            },
        },
        EPSStatus {
            battery1_mv: eps2_battery1_mv,
            battery2_mv: eps2_battery2_mv,
            output_3v3: EPSOutputStatus {
                current_ma: eps2_output_3v3_current_ma.into(),
                status: eps2_output_3v3_status,
            },
            output_5v: EPSOutputStatus {
                current_ma: eps2_output_5v_current_ma.into(),
                status: eps2_output_5v_status,
            },
            output_9v: EPSOutputStatus {
                current_ma: eps2_output_9v_current_ma.into(),
                status: eps2_output_9v_status,
            },
        },
        eps1_node_id,
        eps2_node_id,
        payload_esp_node_id,
    );

    message.serialize(buffer);
    PayloadStatusMessage::len()
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
