#![cfg_attr(not(feature = "wasm"), no_std)]
#![allow(static_mut_refs)]

#[cfg(feature = "wasm")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "wasm")]
use tsify::Tsify;
#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;

use firmware_common_new::can_bus::id::CanBusExtendedId;
use firmware_common_new::can_bus::messages;
use firmware_common_new::can_bus::messages::CanBusMessageEnum;
use firmware_common_new::can_bus::node_types;
use firmware_common_new::can_bus::receiver::CanBusMultiFrameDecoder;
use firmware_common_new::can_bus::sender::CanBusMultiFrameEncoder;

#[cfg_attr(feature = "wasm", derive(Serialize, Deserialize, Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[repr(C)]
pub struct CanBusNodeTypes {
    void_lake: u8,
    amp: u8,
    icarus: u8,
    payload_activation: u8,
    payload_rocket_wifi: u8,
    ozys: u8,
    bulkhead: u8,
    payload_eps1: u8,
    payload_eps2: u8,
    aero_rust: u8,
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = getCanBusNodeTypes))]
pub extern "C" fn get_can_bus_node_types() -> CanBusNodeTypes {
    CanBusNodeTypes {
        void_lake: node_types::VOID_LAKE_NODE_TYPE,
        amp: node_types::AMP_NODE_TYPE,
        icarus: node_types::ICARUS_NODE_TYPE,
        payload_activation: node_types::PAYLOAD_ACTIVATION_NODE_TYPE,
        payload_rocket_wifi: node_types::PAYLOAD_ROCKET_WIFI_NODE_TYPE,
        ozys: node_types::OZYS_NODE_TYPE,
        bulkhead: node_types::BULKHEAD_NODE_TYPE,
        payload_eps1: node_types::PAYLOAD_EPS1_NODE_TYPE,
        payload_eps2: node_types::PAYLOAD_EPS2_NODE_TYPE,
        aero_rust: node_types::AERO_RUST_NODE_TYPE,
    }
}

#[unsafe(no_mangle)]
pub static VOID_LAKE_NODE_TYPE: u8 = node_types::VOID_LAKE_NODE_TYPE;
#[unsafe(no_mangle)]
pub static AMP_NODE_TYPE: u8 = node_types::AMP_NODE_TYPE;
#[unsafe(no_mangle)]
pub static ICARUS_NODE_TYPE: u8 = node_types::ICARUS_NODE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_ACTIVATION_NODE_TYPE: u8 = node_types::PAYLOAD_ACTIVATION_NODE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_ROCKET_WIFI_NODE_TYPE: u8 = node_types::PAYLOAD_ROCKET_WIFI_NODE_TYPE;
#[unsafe(no_mangle)]
pub static OZYS_NODE_TYPE: u8 = node_types::OZYS_NODE_TYPE;
#[unsafe(no_mangle)]
pub static BULKHEAD_NODE_TYPE: u8 = node_types::BULKHEAD_NODE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_EPS1_NODE_TYPE: u8 = node_types::PAYLOAD_EPS1_NODE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_EPS2_NODE_TYPE: u8 = node_types::PAYLOAD_EPS2_NODE_TYPE;
#[unsafe(no_mangle)]
pub static AERO_RUST_NODE_TYPE: u8 = node_types::AERO_RUST_NODE_TYPE;

#[cfg_attr(feature = "wasm", derive(Serialize, Deserialize, Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[repr(C)]
pub struct CanBusMessageTypes {
    reset: u8,
    pre_unix_time: u8,
    unix_time: u8,
    node_status: u8,
    baro_measurement: u8,
    imu_measurement: u8,
    brightness_measurement: u8,
    amp_status: u8,
    amp_control: u8,
    payload_eps_status: u8,
    payload_eps_output_overwrite: u8,
    payload_eps_self_test: u8,
    avionics_status: u8,
    icarus_status: u8,
    data_transfer: u8,
    ack: u8,
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = getCanBusMessageTypes))]
pub extern "C" fn get_can_bus_message_types() -> CanBusMessageTypes {
    CanBusMessageTypes {
        reset: messages::RESET_MESSAGE_TYPE,
        pre_unix_time: messages::PRE_UNIX_TIME_MESSAGE_TYPE,
        unix_time: messages::UNIX_TIME_MESSAGE_TYPE,
        node_status: messages::NODE_STATUS_MESSAGE_TYPE,
        baro_measurement: messages::BARO_MEASUREMENT_MESSAGE_TYPE,
        imu_measurement: messages::IMU_MEASUREMENT_MESSAGE_TYPE,
        brightness_measurement: messages::BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE,
        amp_status: messages::AMP_STATUS_MESSAGE_TYPE,
        amp_control: messages::AMP_CONTROL_MESSAGE_TYPE,
        payload_eps_status: messages::PAYLOAD_EPS_STATUS_MESSAGE_TYPE,
        payload_eps_output_overwrite: messages::PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE,
        payload_eps_self_test: messages::PAYLOAD_EPS_SELF_TEST_MESSAGE_TYPE,
        avionics_status: messages::AVIONICS_STATUS_MESSAGE_TYPE,
        icarus_status: messages::ICARUS_STATUS_MESSAGE_TYPE,
        data_transfer: messages::DATA_TRANSFER_MESSAGE_TYPE,
        ack: messages::ACK_MESSAGE_TYPE,
    }
}

#[unsafe(no_mangle)]
pub static RESET_MESSAGE_TYPE: u8 = messages::RESET_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static PRE_UNIX_TIME_MESSAGE_TYPE: u8 = messages::PRE_UNIX_TIME_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static UNIX_TIME_MESSAGE_TYPE: u8 = messages::UNIX_TIME_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static NODE_STATUS_MESSAGE_TYPE: u8 = messages::NODE_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static BARO_MEASUREMENT_MESSAGE_TYPE: u8 = messages::BARO_MEASUREMENT_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static IMU_MEASUREMENT_MESSAGE_TYPE: u8 = messages::IMU_MEASUREMENT_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE: u8 = messages::BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static AMP_STATUS_MESSAGE_TYPE: u8 = messages::AMP_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static AMP_CONTROL_MESSAGE_TYPE: u8 = messages::AMP_CONTROL_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_EPS_STATUS_MESSAGE_TYPE: u8 = messages::PAYLOAD_EPS_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE: u8 =
    messages::PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static PAYLOAD_EPS_SELF_TEST_MESSAGE_TYPE: u8 = messages::PAYLOAD_EPS_SELF_TEST_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static AVIONICS_STATUS_MESSAGE_TYPE: u8 = messages::AVIONICS_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static ICARUS_STATUS_MESSAGE_TYPE: u8 = messages::ICARUS_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static DATA_TRANSFER_MESSAGE_TYPE: u8 = messages::DATA_TRANSFER_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static ACK_MESSAGE_TYPE: u8 = messages::ACK_MESSAGE_TYPE;

#[cfg_attr(feature = "wasm", derive(Serialize, Deserialize, Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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
#[unsafe(no_mangle)]
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

/// Encodes a CAN bus message into a buffer for transmission.
///
/// # Parameters
/// - `message`: The CAN bus message to encode.
/// - `self_node_type`: The type of the node sending the message.
/// - `self_node_id`: The ID of the node sending the message.
/// - `buffer`: buffer where the encoded message will be written.
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
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = encodeCanBusMessage))]
pub fn encode_can_bus_message_js(
    message: CanBusMessageEnum,
    self_node_type: u8,
    self_node_id: u16,
    buffer: &mut [u8],
) -> CanBusFrames {
    encode_can_bus_message(
        message,
        self_node_type,
        self_node_id,
        buffer.as_mut_ptr(),
        buffer.len(),
    )
}

static mut CAN_DECODER: Option<CanBusMultiFrameDecoder<8>> = None;

#[cfg_attr(feature = "wasm", derive(Serialize, Deserialize, Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[repr(C)]
pub enum ProcessCanBusFrameResult {
    Message {
        timestamp: u64,
        id: CanBusExtendedId,
        crc: u16,
        message: CanBusMessageEnum,
    },
    // the usize does nothing here, it just makes firmware-common-ffi not complain about unsafe zero size type
    Empty(usize),
}

/// Handles the processing of a CAN bus frame to extract a message.
///
/// # Parameters
/// - `timestamp`: The timestamp indicating when the frame was received.
/// - `id`: The ID of the received CAN bus frame.
/// - `data`: A pointer to the buffer containing the frame's data payload.
/// - `data_length`: The size of the data buffer in bytes.
///
/// # Returns
/// - `ProcessCanBusFrameResult`
///     - `Message` if the frame was successfully processed and a complete message was extracted.
///     - `Empty` if the frame is invalid or the message is incomplete (e.g., in the case of multi-frame messages).
#[unsafe(no_mangle)]
pub extern "C" fn process_can_bus_frame(
    timestamp: u64,
    id: u32,
    data: *const u8,
    data_length: usize,
) -> ProcessCanBusFrameResult {
    let data = unsafe { core::slice::from_raw_parts(data, data_length) };
    let frame = (timestamp, id, data);

    let decoder = unsafe {
        if CAN_DECODER.is_none() {
            CAN_DECODER = Some(CanBusMultiFrameDecoder::new())
        }
        CAN_DECODER.as_mut().unwrap()
    };

    match decoder.process_frame(&frame) {
        Some(m) => ProcessCanBusFrameResult::Message {
            timestamp,
            id: CanBusExtendedId::from_raw(id),
            crc: m.data.crc,
            message: m.data.message,
        },
        None => ProcessCanBusFrameResult::Empty(0),
    }
}

/// Handles the processing of a CAN bus frame to extract a message.
///
/// # Parameters
/// - `timestamp`: The timestamp indicating when the frame was received.
/// - `id`: The ID of the received CAN bus frame.
/// - `data`: buffer containing the frame's data payload.
///
/// # Returns
/// - `ProcessCanBusFrameResult`
///     - `Message` if the frame was successfully processed and a complete message was extracted.
///     - `Empty` if the frame is invalid or the message is incomplete (e.g., in the case of multi-frame messages).
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = processCanBusFrame))]
pub fn process_can_bus_frame_js(timestamp: u64, id: u32, data: &[u8]) -> ProcessCanBusFrameResult {
    process_can_bus_frame(timestamp, id, data.as_ptr(), data.len())
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = parseCanBusId))]
pub extern "C" fn parse_can_bus_id(id: u32) -> CanBusExtendedId {
    CanBusExtendedId::from_raw(id)
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = getCanBusMessageType))]
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
#[unsafe(no_mangle)]
pub extern "C" fn can_node_id_from_serial_number(
    serial_number: *const u8,
    serial_number_length: usize,
) -> u16 {
    let serial_number = unsafe { core::slice::from_raw_parts(serial_number, serial_number_length) };
    firmware_common_new::can_bus::id::can_node_id_from_serial_number(serial_number)
}

/// Calculates a CAN node ID from a serial number.
///
/// # Parameters
/// - `serial_number`: serial number
///
/// # Returns
/// The calculated CAN node ID.
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = canNodeIdFromSerialNumber))]
pub fn can_node_id_from_serial_number_js(serial_number: &[u8]) -> u16 {
    can_node_id_from_serial_number(serial_number.as_ptr(), serial_number.len())
}

/// Returns a mask that can be used to filter incoming frames.
///
/// Filter logic: `frame_accepted = (incoming_id & mask) == 0`
///
/// - If the message type of the incoming frame is in `accept_message_types`, the frame will be accepted
/// - If the message type of the incoming frame is not in `accept_message_types`, the frame *MAY OR MAY NOT* be rejected
/// - `ResetMessage` and `UnixTimeMessage` is always accepted even if its not in the `accept_message_types` list
///
/// This is useful when you want to utilize the filter function of the CAN hardware.
///
/// # Parameters
/// - `accept_message_types`: A pointer to the array of message types to accept.
/// - `accept_message_types_length`: The length of the `accept_message_types` array.
#[unsafe(no_mangle)]
pub extern "C" fn create_can_bus_message_type_filter_mask(
    accept_message_types: *const u8,
    accept_message_types_length: usize,
) -> u32 {
    let accept_message_types =
        unsafe { core::slice::from_raw_parts(accept_message_types, accept_message_types_length) };
    firmware_common_new::can_bus::id::create_can_bus_message_type_filter_mask(accept_message_types)
}

/// Returns a mask that can be used to filter incoming frames.
///
/// Filter logic: `frame_accepted = (incoming_id & mask) == 0`
///
/// - If the message type of the incoming frame is in `accept_message_types`, the frame will be accepted
/// - If the message type of the incoming frame is not in `accept_message_types`, the frame *MAY OR MAY NOT* be rejected
/// - `ResetMessage` and `UnixTimeMessage` is always accepted even if its not in the `accept_message_types` list
///
/// This is useful when you want to utilize the filter function of the CAN hardware.
///
/// # Parameters
/// - `accept_message_types`: An array of message types to accept.
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = createCanBusMessageTypeFilterMask))]
pub fn create_can_bus_message_type_filter_mask_js(accept_message_types: &[u8]) -> u32 {
    create_can_bus_message_type_filter_mask(
        accept_message_types.as_ptr(),
        accept_message_types.len(),
    )
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
