#![no_std]

use core::cell::RefCell;

use embassy_sync::lazy_lock::LazyLock;
use firmware_common_new::can_bus::id::CanBusExtendedId;
use firmware_common_new::can_bus::messages::CanBusMessageEnum;
use firmware_common_new::can_bus::receiver::CanBusMultiFrameDecoder;
use firmware_common_new::can_bus::sender::CanBusMultiFrameEncoder;

pub use firmware_common_new::can_bus::node_types;

#[no_mangle]
pub static VOID_LAKE_NODE_TYPE: u8 = node_types::VOID_LAKE_NODE_TYPE;
#[no_mangle]
pub static AMP_NODE_TYPE: u8 = node_types::AMP_NODE_TYPE;
#[no_mangle]
pub static ICARUS_NODE_TYPE: u8 = node_types::ICARUS_NODE_TYPE;
#[no_mangle]
pub static PAYLOAD_ACTIVATION_NODE_TYPE: u8 = node_types::PAYLOAD_ACTIVATION_NODE_TYPE;
#[no_mangle]
pub static PAYLOAD_ROCKET_WIFI_NODE_TYPE: u8 = node_types::PAYLOAD_ROCKET_WIFI_NODE_TYPE;
#[no_mangle]
pub static OZYS_NODE_TYPE: u8 = node_types::OZYS_NODE_TYPE;
#[no_mangle]
pub static BULKHEAD_NODE_TYPE: u8 = node_types::BULKHEAD_NODE_TYPE;
#[no_mangle]
pub static PAYLOAD_EPS1_NODE_TYPE: u8 = node_types::PAYLOAD_EPS1_NODE_TYPE;
#[no_mangle]
pub static PAYLOAD_EPS2_NODE_TYPE: u8 = node_types::PAYLOAD_EPS2_NODE_TYPE;
#[no_mangle]
pub static AERO_RUST_NODE_TYPE: u8 = node_types::AERO_RUST_NODE_TYPE;

#[repr(C)]
pub struct CanBusFrames {
    id: u32,
    len: usize,
    crc: u16,
}

/// Encodes a CAN bus message into a buffer for transmission.
///
/// # Parameters
/// - `message`: The CAN bus message to encode.
/// - `self_node_type`: The type of the node sending the message.
/// - `self_node_id`: The ID of the node sending the message.
/// - `buffer`: A pointer to the buffer where the encoded message will be written.
/// - `buffer_length`: The length of the provided buffer.
///
/// # Returns
/// A `CanBusFrames` struct containing:
/// - `len`: The number of bytes written to the buffer. If the buffer is too small, this will be 0.
/// - `id`: The ID of the CAN bus message.
/// - `crc`: The CRC checksum of the serialized message before encoding, used for comparing against the
///          `crc` field in the received Ack message.
///
/// # Notes
/// The caller is responsible for transmitting the encoded message over the CAN bus in 8-byte chunks.
/// For example, if the returned `len` is 20, the caller should send the following slices of the buffer:
/// - `buffer[0..8]`
/// - `buffer[8..16]`
/// - `buffer[16..20]`
/// All slices should be sent with the same `id` from the return value.
#[no_mangle]
pub extern "C" fn encode_can_bus_message(
    message: CanBusMessageEnum,
    self_node_type: u8,
    self_node_id: u16,
    buffer: *mut u8,
    buffer_length: usize,
) -> CanBusFrames {
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };

    let id = message.get_id(self_node_type, self_node_id);

    let multi_frame_encoder = CanBusMultiFrameEncoder::new(message);
    let crc = multi_frame_encoder.crc;
    let mut i = 0;
    for data in multi_frame_encoder {
        if i + data.len() > buffer.len() {
            return CanBusFrames {
                id: id.into(),
                len: 0,
                crc: 0,
            }; // Buffer too small
        }
        buffer[i..i + data.len()].copy_from_slice(&data);
        i += data.len();
    }

    CanBusFrames {
        id: id.into(),
        len: i,
        crc,
    }
}

static CAN_DECODER: LazyLock<RefCell<CanBusMultiFrameDecoder<8>>> =
    LazyLock::new(|| RefCell::new(CanBusMultiFrameDecoder::new()));

#[repr(C)]
pub struct ReceivedCanBusMessage {
    timestamp: f64,
    id: CanBusExtendedId,
    crc: u16,
    message: CanBusMessageEnum,
}

/// Handles the processing of a CAN bus frame to extract a message.
///
/// # Parameters
/// - `timestamp`: The timestamp indicating when the frame was received.
/// - `id`: The ID of the received CAN bus frame.
/// - `data`: A pointer to the buffer containing the frame's data payload.
/// - `data_length`: The size of the data buffer in bytes.
/// - `result`: A pointer to a `ReceivedCanBusMessage` structure where the extracted message will be stored.
///
/// # Returns
/// - `true` if the frame was successfully processed and a complete message was extracted.
/// - `false` if the frame is invalid or the message is incomplete (e.g., in the case of multi-frame messages).
///
/// # Safety
/// The caller must ensure that the `data` pointer is valid and points to a buffer of at least `data_length` bytes.
/// Additionally, the `result` pointer must be valid and point to a writable `ReceivedCanBusMessage` structure.
///
/// # Notes
/// The contents of `result` are only valid if the function returns `true`.
#[no_mangle]
pub extern "C" fn process_can_bus_frame(
    timestamp: f64,
    id: u32,
    data: *const u8,
    data_length: usize,
    result: *mut ReceivedCanBusMessage,
) -> bool {
    let data = unsafe { core::slice::from_raw_parts(data, data_length) };
    let frame = (timestamp, id, data);

    let mut decoder = CAN_DECODER.get().borrow_mut();

    match decoder.process_frame(&frame) {
        Some(m) => {
            unsafe {
                (*result).timestamp = m.timestamp;
                (*result).id = CanBusExtendedId::from_raw(id);
                (*result).crc = m.data.crc;
                (*result).message = m.data.message;
            }
            true
        }
        None => false,
    }
}

#[no_mangle]
pub extern "C" fn parse_can_bus_id(id: u32) -> CanBusExtendedId {
    CanBusExtendedId::from_raw(id)
}

#[no_mangle]
pub extern "C" fn get_can_bus_message_type(message: CanBusMessageEnum) -> u8 {
    message.get_message_type()
}

/// Calculates a CAN node ID from a serial number.
///
/// # Parameters
/// - `serial_number`: A pointer to the serial number buffer.
/// - `serial_number_length`: The length of the serial number buffer.
///
/// # Returns
/// The calculated CAN node ID.
#[no_mangle]
pub extern "C" fn can_node_id_from_serial_number(
    serial_number: *mut u8,
    serial_number_length: usize,
) -> u16 {
    let serial_number = unsafe { core::slice::from_raw_parts(serial_number, serial_number_length) };
    firmware_common_new::can_bus::id::can_node_id_from_serial_number(serial_number)
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
