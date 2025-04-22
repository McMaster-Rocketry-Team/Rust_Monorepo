#![no_std]

use firmware_common_new::can_bus::id::CanBusExtendedId;
pub use firmware_common_new::can_bus::messages::payload_status::{
    EPSOutputStatus, EPSOutputStatusEnum, EPSStatus,
};
pub use firmware_common_new::can_bus::messages::{CanBusMessage, PayloadStatusMessage};
use firmware_common_new::can_bus::sender::CanBusMultiFrameEncoder;

#[repr(C)]
pub struct CanMessage {
    len: usize,
    id: u32,
}

/// TODO comment
#[no_mangle]
pub extern "C" fn create_payload_status_message(
    buffer: *mut u8,
    buffer_length: usize,
    self_node_type: u8,
    self_node_id: u16,
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
) -> CanMessage {
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

    let id = CanBusExtendedId::from_message(&message, self_node_type, self_node_id);

    let multi_frame_encoder = CanBusMultiFrameEncoder::new(message);
    let mut i = 0;
    for data in multi_frame_encoder {
        if i + data.len() > buffer_length {
            return CanMessage {
                len: 0, // Buffer too small
                id: id.into(),
            };
        }
        buffer[i..i + data.len()].copy_from_slice(&data);
        i += data.len();
    }

    CanMessage {
        len: i,
        id: id.into(),
    }
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
