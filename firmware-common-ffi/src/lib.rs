#![no_std]
#![allow(static_mut_refs)]

use core::mem::{self, MaybeUninit, transmute};

use firmware_common_new::can_bus::messages::payload_eps_status::{
    PayloadEPSOutputStatus, PayloadEPSStatusMessage,
};
use firmware_common_new::can_bus::telemetry::log_multiplexer::LogMultiplexer;
use firmware_common_new::can_bus::telemetry::message_aggregator::CanBusMessageAggregator;

use firmware_common_new::can_bus::id::CanBusExtendedId;
use firmware_common_new::can_bus::messages;
use firmware_common_new::can_bus::messages::CanBusMessageEnum;
use firmware_common_new::can_bus::node_types;
use firmware_common_new::can_bus::receiver::CanBusMultiFrameDecoder;
use firmware_common_new::can_bus::sender::CanBusMultiFrameEncoder;
use firmware_common_new::vlp::client::vlp_decode_ecc;
use firmware_common_new::vlp::packets::VLPDownlinkPacket;

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
pub static AVIONICS_STATUS_MESSAGE_TYPE: u8 = messages::VL_STATUS_MESSAGE_TYPE;
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
    message: *const CanBusMessageEnum,
    self_node_type: u8,
    self_node_id: u16,
    buffer: *mut u8,
    buffer_length: usize,
) -> CanBusFrames {
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
    let message = unsafe { &*message }.clone();

    let id = message.get_id(self_node_type, self_node_id);

    let multi_frame_encoder = CanBusMultiFrameEncoder::new(&message);
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

#[unsafe(no_mangle)]
pub static CAN_BUS_MESSAGE_ENUM_SIZE: usize = mem::size_of::<CanBusMessageEnum>();

struct BtDiagnosticInfra {
    log_multiplexer: LogMultiplexer,
    message_aggregator: CanBusMessageAggregator,
}

#[unsafe(no_mangle)]
pub static BT_DIAGNOSTIC_INFRA_SIZE: usize = mem::size_of::<BtDiagnosticInfra>();

static mut BT_DIAGNOSTIC_INFRA: Option<&'static mut BtDiagnosticInfra> = None;

/// Initializes the bluetooth diagnostic infrastructure.
///
/// This function must be called before using `log_multiplexer_create_chunk` or
/// `message_aggregator_create_chunk`.
///
/// # Parameters
/// - `ptr`: A pointer to memory that will be used to store the diagnostic infrastructure.
///
/// # Safety
///
/// The caller must ensure:
/// - The pointer points to at least `BT_DIAGNOSTIC_INFRA_SIZE` bytes of memory
/// - The pointer is aligned to 8 bytes (typically satisfied by malloc)
/// - The pointer remains valid for the entire lifetime of the program
/// - `init_bluetooth_diagnostic`, `log_multiplexer_create_chunk`,
///   `message_aggregator_create_chunk`, and `process_can_bus_frame` are not
///   invoked concurrently
#[unsafe(no_mangle)]
pub extern "C" fn init_bluetooth_diagnostic(ptr: *mut u8) {
    unsafe {
        if BT_DIAGNOSTIC_INFRA.is_none() {
            let infra: &'static mut MaybeUninit<BtDiagnosticInfra> = transmute(ptr);
            let infra = infra.write(BtDiagnosticInfra {
                log_multiplexer: LogMultiplexer::new(),
                message_aggregator: CanBusMessageAggregator::new(),
            });

            BT_DIAGNOSTIC_INFRA = Some(infra)
        }
    }
}

/// Creates a multiplexed log chunk for sending over bluetooth.
/// The logs come from can bus frames processed by `process_can_bus_frame`
///
/// # Prerequisites
/// - `init_bluetooth_diagnostic` must be called first
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
/// The caller is responsible for ensuring `init_bluetooth_diagnostic`,
/// `log_multiplexer_create_chunk`, `message_aggregator_create_chunk`, and
/// `process_can_bus_frame` are not invoked concurrently
#[unsafe(no_mangle)]
pub extern "C" fn log_multiplexer_create_chunk(buffer: *mut u8, buffer_length: usize) -> usize {
    let log_multiplexer = unsafe {
        if let Some(infra) = &mut BT_DIAGNOSTIC_INFRA {
            &mut infra.log_multiplexer
        } else {
            return 0;
        }
    };

    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
    log_multiplexer.create_chunk(buffer)
}

/// Creates a aggregated can bus message chunk for sending over bluetooth.
/// The messages come from can bus frames processed by `process_can_bus_frame`
///
/// # Prerequisites
/// - `init_bluetooth_diagnostic` must be called first
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
/// The caller is responsible for ensuring `init_bluetooth_diagnostic`,
/// `message_aggregator_create_chunk`, `log_multiplexer_create_chunk`, and
/// `process_can_bus_frame` are not invoked concurrently
#[unsafe(no_mangle)]
pub extern "C" fn message_aggregator_create_chunk(buffer: *mut u8, buffer_length: usize) -> usize {
    let message_aggregator = unsafe {
        if let Some(infra) = &mut BT_DIAGNOSTIC_INFRA {
            &mut infra.message_aggregator
        } else {
            return 0;
        }
    };

    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
    message_aggregator.create_chunk(buffer)
}

static mut CAN_DECODER: Option<CanBusMultiFrameDecoder<8>> = None;

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
/// The caller is responsible for ensuring `init_bluetooth_diagnostic`,
/// `log_multiplexer_create_chunk`, `message_aggregator_create_chunk`, and
/// `process_can_bus_frame` are not invoked concurrently
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

    let infra = unsafe { &mut BT_DIAGNOSTIC_INFRA };

    if let Some(infra) = infra.as_mut() {
        infra.log_multiplexer.process_frame(&frame);
    }
    match decoder.process_frame(&frame) {
        Some(m) => {
            let id = CanBusExtendedId::from_raw(id);
            if let Some(infra) = infra.as_mut() {
                infra
                    .message_aggregator
                    .process_message(&id, &m.data.message, timestamp);
            }
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

#[unsafe(no_mangle)]
pub extern "C" fn parse_can_bus_id(id: u32) -> CanBusExtendedId {
    CanBusExtendedId::from_raw(id)
}

#[unsafe(no_mangle)]
pub extern "C" fn create_log_message_can_bus_id(node_type: u8, node_id: u16) -> CanBusExtendedId {
    CanBusExtendedId::log_message(node_type, node_id)
}

#[unsafe(no_mangle)]
pub extern "C" fn can_bus_extended_id_to_u32(id: CanBusExtendedId) -> u32 {
    id.into()
}

#[unsafe(no_mangle)]
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

#[unsafe(no_mangle)]
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

#[repr(C)]
pub enum DecodeLoraTelemetryResult {
    Success {
        latitude: f64,
        longitude: f64,
        altitude_agl: f32,
    },
    // the usize does nothing here, it just makes firmware-common-ffi not complain about unsafe zero size type
    Invalid(usize),
}

#[unsafe(no_mangle)]
pub extern "C" fn decode_lora_telemetry(
    data: *mut u8,
    data_length: usize,
) -> DecodeLoraTelemetryResult {
    let data = unsafe { core::slice::from_raw_parts_mut(data, data_length) };

    let rx_len = if let Some(rx_len) = vlp_decode_ecc(data) {
        rx_len
    } else {
        return DecodeLoraTelemetryResult::Invalid(0);
    };

    match VLPDownlinkPacket::deserialize(&data[..rx_len]) {
        Some(VLPDownlinkPacket::GPSBeacon(packet)) => DecodeLoraTelemetryResult::Success {
            latitude: packet.lat(),
            longitude: packet.lon(),
            altitude_agl: 0.0,
        },
        Some(VLPDownlinkPacket::Telemetry(packet)) => DecodeLoraTelemetryResult::Success {
            latitude: packet.lat(),
            longitude: packet.lon(),
            altitude_agl: packet.altitude_agl(),
        },
        _ => DecodeLoraTelemetryResult::Invalid(0),
    }
}

/// # Parameters
/// - `firmware_sha512`: A pointer to the SHA512 of the firmware, a 64 bytes array
/// - `signature`: A pointer to the signature, a 64 bytes array
/// - `public_key_str`: A null-terminated string containing the base64-encoded public key
#[unsafe(no_mangle)]
pub extern "C" fn verify_firmware(
    firmware_sha512: *const u8,
    signature: *const u8,
    public_key_str: *const u8,
) -> bool {
    use base64::prelude::*;

    let firmware_sha512 = unsafe { core::slice::from_raw_parts(firmware_sha512, 64) };
    let signature = unsafe { core::slice::from_raw_parts(signature, 64) };

    // Parse the null-terminated string
    let mut len = 0;
    let mut ptr = public_key_str;
    unsafe {
        while *ptr != 0 {
            len += 1;
            ptr = ptr.add(1);
        }
    }
    let public_key_str = unsafe { core::slice::from_raw_parts(public_key_str, len) };

    // Convert to string and decode base64
    let public_key_str = unsafe { core::str::from_utf8_unchecked(public_key_str) };

    let mut public_key = [0u8; 32];
    let decode_key_ok = BASE64_STANDARD
        .decode_slice(public_key_str, &mut public_key)
        .is_ok();
    if !decode_key_ok {
        return false;
    }

    firmware_common_new::bootloader::verify_firmware(
        firmware_sha512.try_into().unwrap(),
        signature.try_into().unwrap(),
        public_key.as_slice().try_into().unwrap(),
    )
    .is_ok()
}

#[cfg(any(target_os = "none", target_os = "espidf"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
