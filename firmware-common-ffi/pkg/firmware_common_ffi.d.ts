/* tslint:disable */
/* eslint-disable */
export function getCanBusNodeTypes(): CanBusNodeTypes;
export function getCanBusMessageTypes(): CanBusMessageTypes;
/**
 * Encodes a CAN bus message into a buffer for transmission.
 *
 * # Parameters
 * - `message`: The CAN bus message to encode.
 * - `self_node_type`: The type of the node sending the message.
 * - `self_node_id`: The ID of the node sending the message.
 * - `buffer`: buffer where the encoded message will be written.
 *
 * # Returns
 * A `CanBusFrames` struct containing:
 * - `len`: The number of bytes written to the buffer. If the buffer is too small, this will be 0.
 * - `id`: The ID of the CAN bus message.
 * - `crc`: The CRC checksum of the serialized message before encoding, used for comparing against the
 *          `crc` field in the received Ack message.
 *
 * # Notes
 * The caller is responsible for transmitting the encoded message over the CAN bus in 8-byte chunks.
 * For example, if the returned `len` is 20, the caller should send the following slices of the buffer:
 * - `buffer[0..8]`
 * - `buffer[8..16]`
 * - `buffer[16..20]`
 * All slices should be sent with the same `id` from the return value.
 */
export function encodeCanBusMessage(message: CanBusMessageEnum, self_node_type: number, self_node_id: number, buffer: Uint8Array): CanBusFrames;
/**
 * Handles the processing of a CAN bus frame to extract a message.
 *
 * # Parameters
 * - `timestamp`: The timestamp indicating when the frame was received.
 * - `id`: The ID of the received CAN bus frame.
 * - `data`: buffer containing the frame's data payload.
 *
 * # Returns
 * - `ProcessCanBusFrameResult`
 *     - `Message` if the frame was successfully processed and a complete message was extracted.
 *     - `Empty` if the frame is invalid or the message is incomplete (e.g., in the case of multi-frame messages).
 */
export function processCanBusFrame(timestamp: bigint, id: number, data: Uint8Array): ProcessCanBusFrameResult;
export function parseCanBusId(id: number): CanBusExtendedId;
export function getCanBusMessageType(message: CanBusMessageEnum): number;
/**
 * Calculates a CAN node ID from a serial number.
 *
 * # Parameters
 * - `serial_number`: serial number
 *
 * # Returns
 * The calculated CAN node ID.
 */
export function canNodeIdFromSerialNumber(serial_number: Uint8Array): number;
/**
 * Returns a mask that can be used to filter incoming frames.
 *
 * Filter logic: `frame_accepted = (incoming_id & mask) == 0`
 *
 * - If the message type of the incoming frame is in `accept_message_types`, the frame will be accepted
 * - If the message type of the incoming frame is not in `accept_message_types`, the frame *MAY OR MAY NOT* be rejected
 * - `ResetMessage` and `UnixTimeMessage` is always accepted even if its not in the `accept_message_types` list
 *
 * This is useful when you want to utilize the filter function of the CAN hardware.
 *
 * # Parameters
 * - `accept_message_types`: An array of message types to accept.
 */
export function createCanBusMessageTypeFilterMask(accept_message_types: Uint8Array): number;
export function newBaroMeasurementMessage(timestamp_us: bigint, pressure: number, temperature: number): BaroMeasurementMessage;
export function baroMeasurementMessageGetPressure(message: BaroMeasurementMessage): number;
export function baroMeasurementMessageGetTemperature(message: BaroMeasurementMessage): number;
export function baroMeasurementMessageGetAltitude(message: BaroMeasurementMessage): number;
export function newBrightnessMeasurementMessage(timestamp_us: bigint, brightness: number): BrightnessMeasurementMessage;
export function brightnessMeasurementMessageGetBrightness(message: BrightnessMeasurementMessage): number;
export function newIcarusStatusMessage(extended_inches: number, servo_current: number, servo_angular_velocity: number): IcarusStatusMessage;
export function icarusStatusMessageGetExtendedInches(message: IcarusStatusMessage): number;
export function icarusStatusMessageGetServoCurrent(message: IcarusStatusMessage): number;
export function newIMUMeasurementMessage(timestamp_us: bigint, accel: Vector3, gyro: Vector3): IMUMeasurementMessage;
export function imuMeasurementMessageGetAcc(message: IMUMeasurementMessage): Vector3;
export function imuMeasurementMessageGetGyro(message: IMUMeasurementMessage): Vector3;
export function newPayloadEPSStatusMessage(battery1_mv: number, battery1_temperature: number, battery2_mv: number, battery2_temperature: number, output_3v3: PayloadEPSOutputStatus, output_5v: PayloadEPSOutputStatus, output_9v: PayloadEPSOutputStatus): PayloadEPSStatusMessage;
export function payloadEPSStatusMessageGetBattery1Temperature(message: PayloadEPSStatusMessage): number;
export function payloadEPSStatusMessageGetBattery2Temperature(message: PayloadEPSStatusMessage): number;
export interface CanBusNodeTypes {
    void_lake: number;
    amp: number;
    icarus: number;
    payload_activation: number;
    payload_rocket_wifi: number;
    ozys: number;
    bulkhead: number;
    payload_eps1: number;
    payload_eps2: number;
    aero_rust: number;
}

export interface CanBusMessageTypes {
    reset: number;
    pre_unix_time: number;
    unix_time: number;
    node_status: number;
    baro_measurement: number;
    imu_measurement: number;
    brightness_measurement: number;
    amp_status: number;
    amp_control: number;
    payload_eps_status: number;
    payload_eps_output_overwrite: number;
    payload_eps_self_test: number;
    avionics_status: number;
    icarus_status: number;
    data_transfer: number;
    ack: number;
    log: number;
}

export interface CanBusFrames {
    id: number;
    len: number;
    crc: number;
}

export type ProcessCanBusFrameResult = { Message: { timestamp: number; id: CanBusExtendedId; crc: number; message: CanBusMessageEnum } } | { Empty: number };

export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

export interface AckMessage {
    /**
     * CRC of the message that was acknowledged
     */
    crc: number;
    /**
     * Node ID of the sender
     */
    node_id: number;
}

export interface AmpControlMessage {
    out1_enable: boolean;
    out2_enable: boolean;
    out3_enable: boolean;
    out4_enable: boolean;
}

export type PowerOutputOverwrite = "NoOverwrite" | "ForceEnabled" | "ForceDisabled";

export interface AmpOverwriteMessage {
    out1: PowerOutputOverwrite;
    out2: PowerOutputOverwrite;
    out3: PowerOutputOverwrite;
    out4: PowerOutputOverwrite;
}

export type PowerOutputStatus = "Disabled" | "PowerGood" | "PowerBad";

export interface AmpOutputStatus {
    overwrote: boolean;
    status: PowerOutputStatus;
}

export interface AmpStatusMessage {
    shared_battery_mv: number;
    out1: AmpOutputStatus;
    out2: AmpOutputStatus;
    out3: AmpOutputStatus;
    out4: AmpOutputStatus;
}

/**
 * may skip stages, may go back to a previous stage
 */
export type FlightStage = "LowPower" | "SelfTest" | "ReadyToLaunch" | "PoweredAscent" | "Coasting" | "DrogueDeployed" | "MainDeployed" | "Landed";

export interface AvionicsStatusMessage {
    flight_stage: FlightStage;
}

export interface BaroMeasurementMessage {
    pressure_raw: number;
    /**
     * Unit: 0.1C, e.g. 250 = 25C
     */
    temperature_raw: number;
    /**
     * Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
     */
    timestamp_us: number;
}

export interface BrightnessMeasurementMessage {
    brightness_raw: number;
    /**
     * Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
     */
    timestamp_us: number;
}

export type DataType = "Firmware" | "Data";

export interface DataTransferMessage {
    data: number[];
    data_len: number;
    /**
     * Message sequence number used to detect duplicates and ensure ordering.
     * Each DataTransferMessage increments message_i by 1 relative to the previous message
     * in the same transfer sequence. Wraps from 255 back to 0.
     */
    sequence_number: number;
    start_of_transfer: boolean;
    end_of_transfer: boolean;
    data_type: DataType;
    destination_node_id: number;
}

export interface IcarusStatusMessage {
    /**
     * Unit: 0.01 inch, e.g. 10 = 0.1 inch
     */
    extended_inches_raw: number;
    /**
     * Unit: 0.01A, e.g. 10 = 0.1A
     */
    servo_current_raw: number;
    /**
     * Unit: deg/s
     */
    servo_angular_velocity: number;
}

export interface IMUMeasurementMessage {
    acc_raw: [number, number, number];
    gyro_raw: [number, number, number];
    /**
     * Measurement timestamp, microseconds since Unix epoch, floored to the nearest us
     */
    timestamp_us: number;
}

export type NodeHealth = "Healthy" | "Warning" | "Error" | "Critical";

export type NodeMode = "Operational" | "Initialization" | "Maintainance" | "Offline";

/**
 * Every node in the network should send this message every 1s.
 * If a node does not send this message for 2s, it is considered offline.
 */
export interface NodeStatusMessage {
    uptime_s: number;
    health: NodeHealth;
    mode: NodeMode;
    /**
     * Node specific status, only the lower 12 bits are used.
     */
    custom_status: number;
}

export interface PayloadEPSOutputOverwriteMessage {
    out_3v3: PowerOutputOverwrite;
    out_5v: PowerOutputOverwrite;
    out_9v: PowerOutputOverwrite;
    /**
     * Node ID of EPS to control
     */
    node_id: number;
}

export interface PayloadEPSSelfTestMessage {
    battery1_ok: boolean;
    battery2_ok: boolean;
    out_3v3_ok: boolean;
    out_5v_ok: boolean;
    out_9v_ok: boolean;
}

export interface PayloadEPSOutputStatus {
    current_ma: number;
    overwrote: boolean;
    status: PowerOutputStatus;
}

export interface PayloadEPSStatusMessage {
    battery1_mv: number;
    /**
     * Unit: 0.1C, e.g. 250 = 25C
     */
    battery1_temperature_raw: number;
    battery2_mv: number;
    /**
     * Unit: 0.1C, e.g. 250 = 25C
     */
    battery2_temperature_raw: number;
    output_3v3: PayloadEPSOutputStatus;
    output_5v: PayloadEPSOutputStatus;
    output_9v: PayloadEPSOutputStatus;
}

export interface ResetMessage {
    node_id: number;
    reset_all: boolean;
    into_bootloader: boolean;
}

export interface UnixTimeMessage {
    /**
     * Current microseconds since Unix epoch, floored to the nearest us
     * 56 representation of it will overflow at year 4254
     */
    timestamp_us: number;
}

export type CanBusMessageEnum = { Reset: ResetMessage } | { PreUnixTime: number } | { UnixTime: UnixTimeMessage } | { NodeStatus: NodeStatusMessage } | { BaroMeasurement: BaroMeasurementMessage } | { IMUMeasurement: IMUMeasurementMessage } | { BrightnessMeasurement: BrightnessMeasurementMessage } | { AmpStatus: AmpStatusMessage } | { AmpOverwrite: AmpOverwriteMessage } | { AmpControl: AmpControlMessage } | { PayloadEPSStatus: PayloadEPSStatusMessage } | { PayloadEPSOutputOverwrite: PayloadEPSOutputOverwriteMessage } | { PayloadEPSSelfTest: PayloadEPSSelfTestMessage } | { AvionicsStatus: AvionicsStatusMessage } | { IcarusStatus: IcarusStatusMessage } | { DataTransfer: DataTransferMessage } | { Ack: AckMessage };

export class CanBusExtendedId {
  free(): void;
  constructor(priority: number, message_type: number, node_type: number, node_id: number);
  static from_raw(raw: number): CanBusExtendedId;
  priority: number;
  message_type: number;
  node_type: number;
  node_id: number;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly encode_can_bus_message: (a: number, b: number, c: number, d: number, e: number, f: number) => void;
  readonly process_can_bus_frame: (a: number, b: bigint, c: number, d: number, e: number) => void;
  readonly can_node_id_from_serial_number: (a: number, b: number) => number;
  readonly create_can_bus_message_type_filter_mask: (a: number, b: number) => number;
  readonly getCanBusNodeTypes: () => any;
  readonly getCanBusMessageTypes: () => any;
  readonly encodeCanBusMessage: (a: any, b: number, c: number, d: number, e: number, f: any) => any;
  readonly processCanBusFrame: (a: bigint, b: number, c: number, d: number) => any;
  readonly getCanBusMessageType: (a: any) => number;
  readonly canNodeIdFromSerialNumber: (a: number, b: number) => number;
  readonly createCanBusMessageTypeFilterMask: (a: number, b: number) => number;
  readonly newBaroMeasurementMessage: (a: bigint, b: number, c: number) => any;
  readonly baroMeasurementMessageGetPressure: (a: any) => number;
  readonly baroMeasurementMessageGetTemperature: (a: any) => number;
  readonly baroMeasurementMessageGetAltitude: (a: any) => number;
  readonly newBrightnessMeasurementMessage: (a: bigint, b: number) => any;
  readonly brightnessMeasurementMessageGetBrightness: (a: any) => number;
  readonly newIcarusStatusMessage: (a: number, b: number, c: number) => any;
  readonly icarusStatusMessageGetExtendedInches: (a: any) => number;
  readonly icarusStatusMessageGetServoCurrent: (a: any) => number;
  readonly newIMUMeasurementMessage: (a: bigint, b: any, c: any) => any;
  readonly imuMeasurementMessageGetAcc: (a: any) => any;
  readonly imuMeasurementMessageGetGyro: (a: any) => any;
  readonly newPayloadEPSStatusMessage: (a: number, b: number, c: number, d: number, e: any, f: any, g: any) => any;
  readonly payloadEPSStatusMessageGetBattery1Temperature: (a: any) => number;
  readonly payloadEPSStatusMessageGetBattery2Temperature: (a: any) => number;
  readonly __wbg_canbusextendedid_free: (a: number, b: number) => void;
  readonly __wbg_get_canbusextendedid_priority: (a: number) => number;
  readonly __wbg_set_canbusextendedid_priority: (a: number, b: number) => void;
  readonly __wbg_get_canbusextendedid_message_type: (a: number) => number;
  readonly __wbg_set_canbusextendedid_message_type: (a: number, b: number) => void;
  readonly __wbg_get_canbusextendedid_node_type: (a: number) => number;
  readonly __wbg_set_canbusextendedid_node_type: (a: number, b: number) => void;
  readonly __wbg_get_canbusextendedid_node_id: (a: number) => number;
  readonly __wbg_set_canbusextendedid_node_id: (a: number, b: number) => void;
  readonly canbusextendedid_new: (a: number, b: number, c: number, d: number) => number;
  readonly canbusextendedid_from_raw: (a: number) => number;
  readonly brightnessmeasurementmessage_new: (a: bigint, b: number) => any;
  readonly parseCanBusId: (a: number) => number;
  readonly brightnessmeasurementmessage_brightness: (a: number) => number;
  readonly __wbindgen_malloc: (a: number, b: number) => number;
  readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
  readonly __wbindgen_exn_store: (a: number) => void;
  readonly __externref_table_alloc: () => number;
  readonly __wbindgen_export_4: WebAssembly.Table;
  readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;
/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
*
* @returns {InitOutput}
*/
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
*
* @returns {Promise<InitOutput>}
*/
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
