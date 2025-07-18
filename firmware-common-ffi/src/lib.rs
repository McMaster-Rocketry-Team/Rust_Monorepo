#![cfg_attr(not(feature = "wasm"), no_std)]
#![allow(static_mut_refs)]

use firmware_common_new::can_bus::messages::baro_measurement::BaroMeasurementMessage;
use firmware_common_new::can_bus::messages::brightness_measurement::BrightnessMeasurementMessage;
use firmware_common_new::can_bus::messages::icarus_status::IcarusStatusMessage;
use firmware_common_new::can_bus::messages::imu_measurement::IMUMeasurementMessage;
use firmware_common_new::can_bus::messages::payload_eps_status::{
    PayloadEPSOutputStatus, PayloadEPSStatusMessage,
};
use firmware_common_new::can_bus::telemetry::log_multiplexer::LogMultiplexer;
use firmware_common_new::can_bus::telemetry::message_aggregator::CanBusMessageAggregator;
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
    amp_speed_bridge: u8,
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
        amp_speed_bridge: node_types::AMP_SPEED_BRIDGE_NODE_TYPE,
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
pub static AMP_SPEED_BRIDGE_NODE_TYPE: u8 = node_types::AMP_SPEED_BRIDGE_NODE_TYPE;
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
    log: u8,
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
        log: messages::LOG_MESSAGE_TYPE,
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
pub static ROCKET_STATE_MESSAGE_TYPE: u8 = messages::ROCKET_STATE_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static ICARUS_STATUS_MESSAGE_TYPE: u8 = messages::ICARUS_STATUS_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static DATA_TRANSFER_MESSAGE_TYPE: u8 = messages::DATA_TRANSFER_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static ACK_MESSAGE_TYPE: u8 = messages::ACK_MESSAGE_TYPE;
#[unsafe(no_mangle)]
pub static LOG_MESSAGE_TYPE: u8 = messages::LOG_MESSAGE_TYPE;

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

static mut LOG_MULTIPLEXER: Option<LogMultiplexer> = None;

/// Creates a multiplexed log chunk for sending over bluetooth.
/// The logs come from can bus frames processed by `process_can_bus_frame`
///
/// # Parameters
/// - `buffer`: A pointer to the buffer where the created chunk will be written to
/// - `buffer_length`: The size of the buffer in bytes.
///
/// # Returns
/// - Length of the created chunk
///
/// # Safety
///
/// The caller is responsible for ensuring `log_multiplexer_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
#[unsafe(no_mangle)]
pub extern "C" fn log_multiplexer_create_chunk(buffer: *mut u8, buffer_length: usize) -> usize {
    let log_multiplexer = unsafe {
        if LOG_MULTIPLEXER.is_none() {
            LOG_MULTIPLEXER = Some(LogMultiplexer::new())
        }
        LOG_MULTIPLEXER.as_mut().unwrap()
    };

    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
    log_multiplexer.create_chunk(buffer)
}

/// Creates a multiplexed log chunk for sending over bluetooth.
/// The logs come from can bus frames processed by `process_can_bus_frame`
///
/// # Parameters
/// - `buffer`: buffer where the created chunk will be written to
///
/// # Returns
/// - Length of the created chunk
///
/// # Safety
///
/// The caller is responsible for ensuring `log_multiplexer_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = logMultiplexerCreateChunk))]
pub fn log_multiplexer_create_chunk_js(buffer: &mut [u8]) -> usize {
    log_multiplexer_create_chunk(buffer.as_mut_ptr(), buffer.len())
}

static mut MESSAGE_AGGREGATOR: Option<CanBusMessageAggregator> = None;

/// Creates a aggregated can bus message chunk for sending over bluetooth.
/// The messages come from can bus frames processed by `process_can_bus_frame`
///
/// # Parameters
/// - `buffer`: A pointer to the buffer where the created chunk will be written to
/// - `buffer_length`: The size of the buffer in bytes.
///
/// # Returns
/// - Length of the created chunk
///
/// # Safety
///
/// The caller is responsible for ensuring `message_aggregator_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
#[unsafe(no_mangle)]
pub extern "C" fn message_aggregator_create_chunk(buffer: *mut u8, buffer_length: usize) -> usize {
    let message_aggregator = unsafe {
        if MESSAGE_AGGREGATOR.is_none() {
            MESSAGE_AGGREGATOR = Some(CanBusMessageAggregator::new())
        }
        MESSAGE_AGGREGATOR.as_mut().unwrap()
    };

    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
    message_aggregator.create_chunk(buffer)
}

/// Creates a aggregated can bus message chunk for sending over bluetooth.
/// The messages come from can bus frames processed by `process_can_bus_frame`
///
/// # Parameters
/// - `buffer`: buffer where the created chunk will be written to
///
/// # Returns
/// - Length of the created chunk
///
/// # Safety
///
/// The caller is responsible for ensuring `message_aggregator_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = messageAggregatorCreateChunk))]
pub fn message_aggregator_create_chunk_js(buffer: &mut [u8]) -> usize {
    message_aggregator_create_chunk(buffer.as_mut_ptr(), buffer.len())
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
///
/// # Safety
///
/// The caller is responsible for ensuring `log_multiplexer_create_chunk`, `message_aggregator_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
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

    let log_multiplexer = unsafe {
        if LOG_MULTIPLEXER.is_none() {
            LOG_MULTIPLEXER = Some(LogMultiplexer::new())
        }
        LOG_MULTIPLEXER.as_mut().unwrap()
    };

    let message_aggregator = unsafe {
        if MESSAGE_AGGREGATOR.is_none() {
            MESSAGE_AGGREGATOR = Some(CanBusMessageAggregator::new())
        }
        MESSAGE_AGGREGATOR.as_mut().unwrap()
    };

    log_multiplexer.process_frame(&frame);
    match decoder.process_frame(&frame) {
        Some(m) => {
            let id = CanBusExtendedId::from_raw(id);
            message_aggregator.process_message(&id, &m.data.message, timestamp);
            ProcessCanBusFrameResult::Message {
                timestamp,
                id,
                crc: m.data.crc,
                message: m.data.message,
            }
        }
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
///
/// # Safety
///
/// The caller is responsible for ensuring `log_multiplexer_create_chunk`, `message_aggregator_create_chunk` and
/// `process_can_bus_frame` is not invoked concurrently
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
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = createLogMessageCanBusId))]
pub extern "C" fn create_log_message_can_bus_id(node_type: u8, node_id: u16) -> CanBusExtendedId {
    CanBusExtendedId::log_message(node_type, node_id)
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = canBusExtendedIdToU32))]
pub extern "C" fn can_bus_extended_id_to_u32(id: CanBusExtendedId) -> u32 {
    id.into()
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

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = newBaroMeasurementMessage))]
pub extern "C" fn new_baro_measurement_message(
    timestamp_us: u64,
    pressure: f32,
    temperature: f32,
) -> BaroMeasurementMessage {
    BaroMeasurementMessage::new(timestamp_us, pressure, temperature)
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = baroMeasurementMessageGetPressure))]
pub extern "C" fn baro_measurement_message_get_pressure(message: &BaroMeasurementMessage) -> f32 {
    message.pressure()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = baroMeasurementMessageGetTemperature))]
pub extern "C" fn baro_measurement_message_get_temperature(
    message: &BaroMeasurementMessage,
) -> f32 {
    message.temperature()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = baroMeasurementMessageGetAltitude))]
pub extern "C" fn baro_measurement_message_get_altitude(message: &BaroMeasurementMessage) -> f32 {
    message.altitude()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = newBrightnessMeasurementMessage))]
pub extern "C" fn new_brightness_measurement_message(
    timestamp_us: u64,
    brightness: f32,
) -> BrightnessMeasurementMessage {
    BrightnessMeasurementMessage::new(timestamp_us, brightness)
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = brightnessMeasurementMessageGetBrightness))]
pub extern "C" fn brightness_measurement_message_get_brightness(
    message: &BrightnessMeasurementMessage,
) -> f32 {
    message.brightness_lux()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = newIcarusStatusMessage))]
pub extern "C" fn new_icarus_status_message(
    extended_inches: f32,
    servo_current: f32,
    servo_angular_velocity: i16,
) -> IcarusStatusMessage {
    IcarusStatusMessage::new(extended_inches, servo_current, servo_angular_velocity)
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = icarusStatusMessageGetExtendedInches))]
pub extern "C" fn icarus_status_message_get_extended_inches(message: &IcarusStatusMessage) -> f32 {
    message.extended_inches()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = icarusStatusMessageGetServoCurrent))]
pub extern "C" fn icarus_status_message_get_servo_current(message: &IcarusStatusMessage) -> f32 {
    message.servo_current()
}

#[cfg_attr(feature = "wasm", derive(Serialize, Deserialize, Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[repr(C)]
pub struct Vector3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Into<[f32; 3]> for Vector3 {
    fn into(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

impl From<[f32; 3]> for Vector3 {
    fn from(value: [f32; 3]) -> Self {
        Vector3 {
            x: value[0],
            y: value[1],
            z: value[2],
        }
    }
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = newIMUMeasurementMessage))]
pub extern "C" fn new_imu_measurement_message(
    timestamp_us: u64,
    accel: Vector3,
    gyro: Vector3,
) -> IMUMeasurementMessage {
    IMUMeasurementMessage::new(timestamp_us, &accel.into(), &gyro.into())
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = imuMeasurementMessageGetAcc))]
pub extern "C" fn imu_measurement_message_get_acc(message: &IMUMeasurementMessage) -> Vector3 {
    message.acc().into()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = imuMeasurementMessageGetGyro))]
pub extern "C" fn imu_measurement_message_get_gyro(message: &IMUMeasurementMessage) -> Vector3 {
    message.gyro().into()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = newPayloadEPSStatusMessage))]
pub extern "C" fn new_payload_eps_status_message(
    battery1_mv: u16,
    battery1_temperature: f32,
    battery2_mv: u16,
    battery2_temperature: f32,
    output_3v3: PayloadEPSOutputStatus,
    output_5v: PayloadEPSOutputStatus,
    output_9v: PayloadEPSOutputStatus,
) -> PayloadEPSStatusMessage {
    PayloadEPSStatusMessage::new(
        battery1_mv,
        battery1_temperature,
        battery2_mv,
        battery2_temperature,
        output_3v3,
        output_5v,
        output_9v,
    )
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = payloadEPSStatusMessageGetBattery1Temperature))]
pub extern "C" fn payload_eps_status_message_get_battery1_temperature(
    message: &PayloadEPSStatusMessage,
) -> f32 {
    message.battery1_temperature()
}

#[cfg_attr(not(feature = "wasm"), unsafe(no_mangle))]
#[cfg_attr(feature = "wasm", wasm_bindgen(js_name = payloadEPSStatusMessageGetBattery2Temperature))]
pub extern "C" fn payload_eps_status_message_get_battery2_temperature(
    message: &PayloadEPSStatusMessage,
) -> f32 {
    message.battery2_temperature()
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
