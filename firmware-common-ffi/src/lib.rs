#![no_std]

use firmware_common_new::can_bus::id::CanBusExtendedId;
pub use firmware_common_new::can_bus::messages::payload_status::{EPSOutputStatus, EPSStatus};
use firmware_common_new::can_bus::messages::CanBusMessageEnum;
pub use firmware_common_new::can_bus::messages::{CanBusMessage, PayloadStatusMessage};
use firmware_common_new::can_bus::sender::CanBusMultiFrameEncoder;

#[repr(C)]
pub struct CanMessage {
    id: u32,
    len: usize,
}

/// TODO comment
#[no_mangle]
pub extern "C" fn create_can_bus_message(
    buffer: *mut u8,
    buffer_length: usize,
    message: CanBusMessageEnum,
    self_node_type: u8,
    self_node_id: u16,
) -> CanMessage {
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };

    match message {
        CanBusMessageEnum::UnixTime(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::NodeStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::Reset(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::BaroMeasurement(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::IMUMeasurement(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::TempuratureMeasurement(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::AmpStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::AmpControl(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::PayloadStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::PayloadControl(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::PayloadSelfTest(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::AvionicsStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::IcarusStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::BulkheadStatus(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::DataTransfer(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
        CanBusMessageEnum::Ack(m) => {
            inner_create_can_bus_message(buffer, m, self_node_type, self_node_id)
        }
    }
}

fn inner_create_can_bus_message(
    buffer: &mut [u8],
    message: impl CanBusMessage,
    self_node_type: u8,
    self_node_id: u16,
) -> CanMessage {
    let id = CanBusExtendedId::from_message(&message, self_node_type, self_node_id);

    let multi_frame_encoder = CanBusMultiFrameEncoder::new(message);
    let mut i = 0;
    for data in multi_frame_encoder {
        if i + data.len() > buffer.len() {
            return CanMessage {
                id: id.into(),
                len: 0,
            }; // Buffer too small
        }
        buffer[i..i + data.len()].copy_from_slice(&data);
        i += data.len();
    }

    CanMessage {
        id: id.into(),
        len: i,
    }
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
