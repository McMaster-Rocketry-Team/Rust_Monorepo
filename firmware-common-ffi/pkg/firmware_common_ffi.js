let wasm;

let WASM_VECTOR_LEN = 0;

let cachedUint8ArrayMemory0 = null;

function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

const cachedTextEncoder = (typeof TextEncoder !== 'undefined' ? new TextEncoder('utf-8') : { encode: () => { throw Error('TextEncoder not available') } } );

const encodeString = (typeof cachedTextEncoder.encodeInto === 'function'
    ? function (arg, view) {
    return cachedTextEncoder.encodeInto(arg, view);
}
    : function (arg, view) {
    const buf = cachedTextEncoder.encode(arg);
    view.set(buf);
    return {
        read: arg.length,
        written: buf.length
    };
});

function passStringToWasm0(arg, malloc, realloc) {

    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }

    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = encodeString(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

let cachedDataViewMemory0 = null;

function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_export_4.set(idx, obj);
    return idx;
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        const idx = addToExternrefTable0(e);
        wasm.__wbindgen_exn_store(idx);
    }
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

function debugString(val) {
    // primitive types
    const type = typeof val;
    if (type == 'number' || type == 'boolean' || val == null) {
        return  `${val}`;
    }
    if (type == 'string') {
        return `"${val}"`;
    }
    if (type == 'symbol') {
        const description = val.description;
        if (description == null) {
            return 'Symbol';
        } else {
            return `Symbol(${description})`;
        }
    }
    if (type == 'function') {
        const name = val.name;
        if (typeof name == 'string' && name.length > 0) {
            return `Function(${name})`;
        } else {
            return 'Function';
        }
    }
    // objects
    if (Array.isArray(val)) {
        const length = val.length;
        let debug = '[';
        if (length > 0) {
            debug += debugString(val[0]);
        }
        for(let i = 1; i < length; i++) {
            debug += ', ' + debugString(val[i]);
        }
        debug += ']';
        return debug;
    }
    // Test for built-in
    const builtInMatches = /\[object ([^\]]+)\]/.exec(toString.call(val));
    let className;
    if (builtInMatches && builtInMatches.length > 1) {
        className = builtInMatches[1];
    } else {
        // Failed to match the standard '[object ClassName]'
        return toString.call(val);
    }
    if (className == 'Object') {
        // we're a user defined class or Object
        // JSON.stringify avoids problems with cycles, and is generally much
        // easier than looping through ownProperties of `val`.
        try {
            return 'Object(' + JSON.stringify(val) + ')';
        } catch (_) {
            return 'Object';
        }
    }
    // errors
    if (val instanceof Error) {
        return `${val.name}: ${val.message}\n${val.stack}`;
    }
    // TODO we could test for more things here, like `Set`s and `Map`s.
    return className;
}

const cachedTextDecoder = (typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-8', { ignoreBOM: true, fatal: true }) : { decode: () => { throw Error('TextDecoder not available') } } );

if (typeof TextDecoder !== 'undefined') { cachedTextDecoder.decode(); };

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}
/**
 * @returns {CanBusNodeTypes}
 */
export function getCanBusNodeTypes() {
    const ret = wasm.getCanBusNodeTypes();
    return ret;
}

/**
 * @returns {CanBusMessageTypes}
 */
export function getCanBusMessageTypes() {
    const ret = wasm.getCanBusMessageTypes();
    return ret;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}
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
 * @param {CanBusMessageEnum} message
 * @param {number} self_node_type
 * @param {number} self_node_id
 * @param {Uint8Array} buffer
 * @returns {CanBusFrames}
 */
export function encodeCanBusMessage(message, self_node_type, self_node_id, buffer) {
    var ptr0 = passArray8ToWasm0(buffer, wasm.__wbindgen_malloc);
    var len0 = WASM_VECTOR_LEN;
    const ret = wasm.encodeCanBusMessage(message, self_node_type, self_node_id, ptr0, len0, buffer);
    return ret;
}

/**
 * Creates a multiplexed log chunk for sending over bluetooth.
 * The logs come from can bus frames processed by `process_can_bus_frame`
 *
 * # Parameters
 * - `buffer`: buffer where the created chunk will be written to
 *
 * # Returns
 * - Length of the created chunk
 *
 * # Safety
 *
 * The caller is responsible for ensuring `log_multiplexer_create_chunk` and
 * `process_can_bus_frame` is not invoked concurrently
 * @param {Uint8Array} buffer
 * @returns {number}
 */
export function logMultiplexerCreateChunk(buffer) {
    var ptr0 = passArray8ToWasm0(buffer, wasm.__wbindgen_malloc);
    var len0 = WASM_VECTOR_LEN;
    const ret = wasm.logMultiplexerCreateChunk(ptr0, len0, buffer);
    return ret >>> 0;
}

/**
 * Creates a aggregated can bus message chunk for sending over bluetooth.
 * The messages come from can bus frames processed by `process_can_bus_frame`
 *
 * # Parameters
 * - `buffer`: buffer where the created chunk will be written to
 *
 * # Returns
 * - Length of the created chunk
 *
 * # Safety
 *
 * The caller is responsible for ensuring `message_aggregator_create_chunk` and
 * `process_can_bus_frame` is not invoked concurrently
 * @param {Uint8Array} buffer
 * @returns {number}
 */
export function messageAggregatorCreateChunk(buffer) {
    var ptr0 = passArray8ToWasm0(buffer, wasm.__wbindgen_malloc);
    var len0 = WASM_VECTOR_LEN;
    const ret = wasm.messageAggregatorCreateChunk(ptr0, len0, buffer);
    return ret >>> 0;
}

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
 *
 * # Safety
 *
 * The caller is responsible for ensuring `log_multiplexer_create_chunk`, `message_aggregator_create_chunk` and
 * `process_can_bus_frame` is not invoked concurrently
 * @param {bigint} timestamp
 * @param {number} id
 * @param {Uint8Array} data
 * @returns {ProcessCanBusFrameResult}
 */
export function processCanBusFrame(timestamp, id, data) {
    const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.processCanBusFrame(timestamp, id, ptr0, len0);
    return ret;
}

/**
 * @param {number} id
 * @returns {CanBusExtendedId}
 */
export function parseCanBusId(id) {
    const ret = wasm.canbusextendedid_from_raw(id);
    return CanBusExtendedId.__wrap(ret);
}

/**
 * @param {number} node_type
 * @param {number} node_id
 * @returns {CanBusExtendedId}
 */
export function createLogMessageCanBusId(node_type, node_id) {
    const ret = wasm.canbusextendedid_log_message(node_type, node_id);
    return CanBusExtendedId.__wrap(ret);
}

function _assertClass(instance, klass) {
    if (!(instance instanceof klass)) {
        throw new Error(`expected instance of ${klass.name}`);
    }
}
/**
 * @param {CanBusExtendedId} id
 * @returns {number}
 */
export function canBusExtendedIdToU32(id) {
    _assertClass(id, CanBusExtendedId);
    var ptr0 = id.__destroy_into_raw();
    const ret = wasm.canBusExtendedIdToU32(ptr0);
    return ret >>> 0;
}

/**
 * @param {CanBusMessageEnum} message
 * @returns {number}
 */
export function getCanBusMessageType(message) {
    const ret = wasm.getCanBusMessageType(message);
    return ret;
}

/**
 * Calculates a CAN node ID from a serial number.
 *
 * # Parameters
 * - `serial_number`: serial number
 *
 * # Returns
 * The calculated CAN node ID.
 * @param {Uint8Array} serial_number
 * @returns {number}
 */
export function canNodeIdFromSerialNumber(serial_number) {
    const ptr0 = passArray8ToWasm0(serial_number, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.canNodeIdFromSerialNumber(ptr0, len0);
    return ret;
}

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
 * @param {Uint8Array} accept_message_types
 * @returns {number}
 */
export function createCanBusMessageTypeFilterMask(accept_message_types) {
    const ptr0 = passArray8ToWasm0(accept_message_types, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.createCanBusMessageTypeFilterMask(ptr0, len0);
    return ret >>> 0;
}

/**
 * @param {bigint} timestamp_us
 * @param {number} pressure
 * @param {number} temperature
 * @returns {BaroMeasurementMessage}
 */
export function newBaroMeasurementMessage(timestamp_us, pressure, temperature) {
    const ret = wasm.newBaroMeasurementMessage(timestamp_us, pressure, temperature);
    return ret;
}

/**
 * @param {BaroMeasurementMessage} message
 * @returns {number}
 */
export function baroMeasurementMessageGetPressure(message) {
    const ret = wasm.baroMeasurementMessageGetPressure(message);
    return ret;
}

/**
 * @param {BaroMeasurementMessage} message
 * @returns {number}
 */
export function baroMeasurementMessageGetTemperature(message) {
    const ret = wasm.baroMeasurementMessageGetTemperature(message);
    return ret;
}

/**
 * @param {BaroMeasurementMessage} message
 * @returns {number}
 */
export function baroMeasurementMessageGetAltitude(message) {
    const ret = wasm.baroMeasurementMessageGetAltitude(message);
    return ret;
}

/**
 * @param {bigint} timestamp_us
 * @param {number} brightness
 * @returns {BrightnessMeasurementMessage}
 */
export function newBrightnessMeasurementMessage(timestamp_us, brightness) {
    const ret = wasm.newBrightnessMeasurementMessage(timestamp_us, brightness);
    return ret;
}

/**
 * @param {BrightnessMeasurementMessage} message
 * @returns {number}
 */
export function brightnessMeasurementMessageGetBrightness(message) {
    const ret = wasm.brightnessMeasurementMessageGetBrightness(message);
    return ret;
}

/**
 * @param {number} extended_inches
 * @param {number} servo_current
 * @param {number} servo_angular_velocity
 * @returns {IcarusStatusMessage}
 */
export function newIcarusStatusMessage(extended_inches, servo_current, servo_angular_velocity) {
    const ret = wasm.newIcarusStatusMessage(extended_inches, servo_current, servo_angular_velocity);
    return ret;
}

/**
 * @param {IcarusStatusMessage} message
 * @returns {number}
 */
export function icarusStatusMessageGetExtendedInches(message) {
    const ret = wasm.icarusStatusMessageGetExtendedInches(message);
    return ret;
}

/**
 * @param {IcarusStatusMessage} message
 * @returns {number}
 */
export function icarusStatusMessageGetServoCurrent(message) {
    const ret = wasm.icarusStatusMessageGetServoCurrent(message);
    return ret;
}

/**
 * @param {bigint} timestamp_us
 * @param {Vector3} accel
 * @param {Vector3} gyro
 * @returns {IMUMeasurementMessage}
 */
export function newIMUMeasurementMessage(timestamp_us, accel, gyro) {
    const ret = wasm.newIMUMeasurementMessage(timestamp_us, accel, gyro);
    return ret;
}

/**
 * @param {IMUMeasurementMessage} message
 * @returns {Vector3}
 */
export function imuMeasurementMessageGetAcc(message) {
    const ret = wasm.imuMeasurementMessageGetAcc(message);
    return ret;
}

/**
 * @param {IMUMeasurementMessage} message
 * @returns {Vector3}
 */
export function imuMeasurementMessageGetGyro(message) {
    const ret = wasm.imuMeasurementMessageGetGyro(message);
    return ret;
}

/**
 * @param {number} battery1_mv
 * @param {number} battery1_temperature
 * @param {number} battery2_mv
 * @param {number} battery2_temperature
 * @param {PayloadEPSOutputStatus} output_3v3
 * @param {PayloadEPSOutputStatus} output_5v
 * @param {PayloadEPSOutputStatus} output_9v
 * @returns {PayloadEPSStatusMessage}
 */
export function newPayloadEPSStatusMessage(battery1_mv, battery1_temperature, battery2_mv, battery2_temperature, output_3v3, output_5v, output_9v) {
    const ret = wasm.newPayloadEPSStatusMessage(battery1_mv, battery1_temperature, battery2_mv, battery2_temperature, output_3v3, output_5v, output_9v);
    return ret;
}

/**
 * @param {PayloadEPSStatusMessage} message
 * @returns {number}
 */
export function payloadEPSStatusMessageGetBattery1Temperature(message) {
    const ret = wasm.payloadEPSStatusMessageGetBattery1Temperature(message);
    return ret;
}

/**
 * @param {PayloadEPSStatusMessage} message
 * @returns {number}
 */
export function payloadEPSStatusMessageGetBattery2Temperature(message) {
    const ret = wasm.payloadEPSStatusMessageGetBattery2Temperature(message);
    return ret;
}

const BrightnessMeasurementMessageFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_brightnessmeasurementmessage_free(ptr >>> 0, 1));

export class BrightnessMeasurementMessage {

    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        BrightnessMeasurementMessageFinalization.unregister(this);
        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_brightnessmeasurementmessage_free(ptr, 0);
    }
    /**
     * @param {bigint} timestamp_us
     * @param {number} brightness
     * @returns {BrightnessMeasurementMessage}
     */
    static new(timestamp_us, brightness) {
        const ret = wasm.brightnessmeasurementmessage_new(timestamp_us, brightness);
        return ret;
    }
    /**
     * Brightness in Lux
     * @returns {number}
     */
    brightness() {
        const ret = wasm.brightnessmeasurementmessage_brightness(this.__wbg_ptr);
        return ret;
    }
}

const CanBusExtendedIdFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_canbusextendedid_free(ptr >>> 0, 1));

export class CanBusExtendedId {

    static __wrap(ptr) {
        ptr = ptr >>> 0;
        const obj = Object.create(CanBusExtendedId.prototype);
        obj.__wbg_ptr = ptr;
        CanBusExtendedIdFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }

    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CanBusExtendedIdFinalization.unregister(this);
        return ptr;
    }

    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_canbusextendedid_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    get priority() {
        const ret = wasm.__wbg_get_canbusextendedid_priority(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set priority(arg0) {
        wasm.__wbg_set_canbusextendedid_priority(this.__wbg_ptr, arg0);
    }
    /**
     * @returns {number}
     */
    get message_type() {
        const ret = wasm.__wbg_get_canbusextendedid_message_type(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set message_type(arg0) {
        wasm.__wbg_set_canbusextendedid_message_type(this.__wbg_ptr, arg0);
    }
    /**
     * @returns {number}
     */
    get node_type() {
        const ret = wasm.__wbg_get_canbusextendedid_node_type(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set node_type(arg0) {
        wasm.__wbg_set_canbusextendedid_node_type(this.__wbg_ptr, arg0);
    }
    /**
     * @returns {number}
     */
    get node_id() {
        const ret = wasm.__wbg_get_canbusextendedid_node_id(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set node_id(arg0) {
        wasm.__wbg_set_canbusextendedid_node_id(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} priority
     * @param {number} message_type
     * @param {number} node_type
     * @param {number} node_id
     */
    constructor(priority, message_type, node_type, node_id) {
        const ret = wasm.canbusextendedid_new(priority, message_type, node_type, node_id);
        this.__wbg_ptr = ret >>> 0;
        CanBusExtendedIdFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} raw
     * @returns {CanBusExtendedId}
     */
    static from_raw(raw) {
        const ret = wasm.canbusextendedid_from_raw(raw);
        return CanBusExtendedId.__wrap(ret);
    }
    /**
     * @param {number} node_type
     * @param {number} node_id
     * @returns {CanBusExtendedId}
     */
    static log_message(node_type, node_id) {
        const ret = wasm.canbusextendedid_log_message(node_type, node_id);
        return CanBusExtendedId.__wrap(ret);
    }
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);

            } catch (e) {
                if (module.headers.get('Content-Type') != 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);

    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };

        } else {
            return instance;
        }
    }
}

function __wbg_get_imports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbg_String_8f0eb39a4a4c2f66 = function(arg0, arg1) {
        const ret = String(arg1);
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
    };
    imports.wbg.__wbg_buffer_609cc3eee51ed158 = function(arg0) {
        const ret = arg0.buffer;
        return ret;
    };
    imports.wbg.__wbg_call_672a4d21634d4a24 = function() { return handleError(function (arg0, arg1) {
        const ret = arg0.call(arg1);
        return ret;
    }, arguments) };
    imports.wbg.__wbg_done_769e5ede4b31c67b = function(arg0) {
        const ret = arg0.done;
        return ret;
    };
    imports.wbg.__wbg_entries_3265d4158b33e5dc = function(arg0) {
        const ret = Object.entries(arg0);
        return ret;
    };
    imports.wbg.__wbg_get_67b2ba62fc30de12 = function() { return handleError(function (arg0, arg1) {
        const ret = Reflect.get(arg0, arg1);
        return ret;
    }, arguments) };
    imports.wbg.__wbg_get_b9b93047fe3cf45b = function(arg0, arg1) {
        const ret = arg0[arg1 >>> 0];
        return ret;
    };
    imports.wbg.__wbg_getwithrefkey_1dc361bd10053bfe = function(arg0, arg1) {
        const ret = arg0[arg1];
        return ret;
    };
    imports.wbg.__wbg_instanceof_ArrayBuffer_e14585432e3737fc = function(arg0) {
        let result;
        try {
            result = arg0 instanceof ArrayBuffer;
        } catch (_) {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_instanceof_Uint8Array_17156bcf118086a9 = function(arg0) {
        let result;
        try {
            result = arg0 instanceof Uint8Array;
        } catch (_) {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_isArray_a1eab7e0d067391b = function(arg0) {
        const ret = Array.isArray(arg0);
        return ret;
    };
    imports.wbg.__wbg_isSafeInteger_343e2beeeece1bb0 = function(arg0) {
        const ret = Number.isSafeInteger(arg0);
        return ret;
    };
    imports.wbg.__wbg_iterator_9a24c88df860dc65 = function() {
        const ret = Symbol.iterator;
        return ret;
    };
    imports.wbg.__wbg_length_a446193dc22c12f8 = function(arg0) {
        const ret = arg0.length;
        return ret;
    };
    imports.wbg.__wbg_length_e2d2a49132c1b256 = function(arg0) {
        const ret = arg0.length;
        return ret;
    };
    imports.wbg.__wbg_new_405e22f390576ce2 = function() {
        const ret = new Object();
        return ret;
    };
    imports.wbg.__wbg_new_78feb108b6472713 = function() {
        const ret = new Array();
        return ret;
    };
    imports.wbg.__wbg_new_a12002a7f91c75be = function(arg0) {
        const ret = new Uint8Array(arg0);
        return ret;
    };
    imports.wbg.__wbg_next_25feadfc0913fea9 = function(arg0) {
        const ret = arg0.next;
        return ret;
    };
    imports.wbg.__wbg_next_6574e1a8a62d1055 = function() { return handleError(function (arg0) {
        const ret = arg0.next();
        return ret;
    }, arguments) };
    imports.wbg.__wbg_set_37837023f3d740e8 = function(arg0, arg1, arg2) {
        arg0[arg1 >>> 0] = arg2;
    };
    imports.wbg.__wbg_set_3f1d0b984ed272ed = function(arg0, arg1, arg2) {
        arg0[arg1] = arg2;
    };
    imports.wbg.__wbg_set_65595bdd868b3009 = function(arg0, arg1, arg2) {
        arg0.set(arg1, arg2 >>> 0);
    };
    imports.wbg.__wbg_value_cd1ffa7b1ab794f1 = function(arg0) {
        const ret = arg0.value;
        return ret;
    };
    imports.wbg.__wbindgen_as_number = function(arg0) {
        const ret = +arg0;
        return ret;
    };
    imports.wbg.__wbindgen_bigint_from_u64 = function(arg0) {
        const ret = BigInt.asUintN(64, arg0);
        return ret;
    };
    imports.wbg.__wbindgen_bigint_get_as_i64 = function(arg0, arg1) {
        const v = arg1;
        const ret = typeof(v) === 'bigint' ? v : undefined;
        getDataViewMemory0().setBigInt64(arg0 + 8 * 1, isLikeNone(ret) ? BigInt(0) : ret, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, !isLikeNone(ret), true);
    };
    imports.wbg.__wbindgen_boolean_get = function(arg0) {
        const v = arg0;
        const ret = typeof(v) === 'boolean' ? (v ? 1 : 0) : 2;
        return ret;
    };
    imports.wbg.__wbindgen_copy_to_typed_array = function(arg0, arg1, arg2) {
        new Uint8Array(arg2.buffer, arg2.byteOffset, arg2.byteLength).set(getArrayU8FromWasm0(arg0, arg1));
    };
    imports.wbg.__wbindgen_debug_string = function(arg0, arg1) {
        const ret = debugString(arg1);
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
    };
    imports.wbg.__wbindgen_error_new = function(arg0, arg1) {
        const ret = new Error(getStringFromWasm0(arg0, arg1));
        return ret;
    };
    imports.wbg.__wbindgen_in = function(arg0, arg1) {
        const ret = arg0 in arg1;
        return ret;
    };
    imports.wbg.__wbindgen_init_externref_table = function() {
        const table = wasm.__wbindgen_export_4;
        const offset = table.grow(4);
        table.set(0, undefined);
        table.set(offset + 0, undefined);
        table.set(offset + 1, null);
        table.set(offset + 2, true);
        table.set(offset + 3, false);
        ;
    };
    imports.wbg.__wbindgen_is_bigint = function(arg0) {
        const ret = typeof(arg0) === 'bigint';
        return ret;
    };
    imports.wbg.__wbindgen_is_function = function(arg0) {
        const ret = typeof(arg0) === 'function';
        return ret;
    };
    imports.wbg.__wbindgen_is_object = function(arg0) {
        const val = arg0;
        const ret = typeof(val) === 'object' && val !== null;
        return ret;
    };
    imports.wbg.__wbindgen_is_string = function(arg0) {
        const ret = typeof(arg0) === 'string';
        return ret;
    };
    imports.wbg.__wbindgen_is_undefined = function(arg0) {
        const ret = arg0 === undefined;
        return ret;
    };
    imports.wbg.__wbindgen_jsval_eq = function(arg0, arg1) {
        const ret = arg0 === arg1;
        return ret;
    };
    imports.wbg.__wbindgen_jsval_loose_eq = function(arg0, arg1) {
        const ret = arg0 == arg1;
        return ret;
    };
    imports.wbg.__wbindgen_memory = function() {
        const ret = wasm.memory;
        return ret;
    };
    imports.wbg.__wbindgen_number_get = function(arg0, arg1) {
        const obj = arg1;
        const ret = typeof(obj) === 'number' ? obj : undefined;
        getDataViewMemory0().setFloat64(arg0 + 8 * 1, isLikeNone(ret) ? 0 : ret, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, !isLikeNone(ret), true);
    };
    imports.wbg.__wbindgen_number_new = function(arg0) {
        const ret = arg0;
        return ret;
    };
    imports.wbg.__wbindgen_string_get = function(arg0, arg1) {
        const obj = arg1;
        const ret = typeof(obj) === 'string' ? obj : undefined;
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
    };
    imports.wbg.__wbindgen_string_new = function(arg0, arg1) {
        const ret = getStringFromWasm0(arg0, arg1);
        return ret;
    };
    imports.wbg.__wbindgen_throw = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };

    return imports;
}

function __wbg_init_memory(imports, memory) {

}

function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    __wbg_init.__wbindgen_wasm_module = module;
    cachedDataViewMemory0 = null;
    cachedUint8ArrayMemory0 = null;


    wasm.__wbindgen_start();
    return wasm;
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (typeof module !== 'undefined') {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();

    __wbg_init_memory(imports);

    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }

    const instance = new WebAssembly.Instance(module, imports);

    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (typeof module_or_path !== 'undefined') {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (typeof module_or_path === 'undefined') {
        module_or_path = new URL('firmware_common_ffi_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    __wbg_init_memory(imports);

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync };
export default __wbg_init;
