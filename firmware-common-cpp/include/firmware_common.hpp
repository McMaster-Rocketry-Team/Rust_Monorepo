#pragma once

// Hand-written C++ port of firmware-common-new/src/can_bus. Nothing generates
// this file, so it has to be updated by hand whenever a CAN message changes:
//
//   1. edit the message in firmware-common-new/src/can_bus/messages/
//   2. `cargo test --lib can_bus` in firmware-common-new, which rewrites the
//      golden vectors in firmware-common-new/can_bus_reference_data/
//   3. mirror the change here
//   4. `cmake --build build && ctest` in firmware-common-cpp to check the port
//      still produces byte-identical frames
//
// Bit layouts follow Rust's packed_struct with bit_numbering = msb0 and
// endian = msb. A struct whose fields are shorter than its declared
// size_bytes is padded at the end.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <variant>
#include <optional>
#include <type_traits>

namespace firmware_common {
namespace can_bus {

    // Helper for type punning (safe alternative to union)
    template <typename To, typename From>
    inline To bit_cast(const From& src) noexcept {
        static_assert(sizeof(To) == sizeof(From), "bit_cast sizes must match");
        To dst;
        std::memcpy(&dst, &src, sizeof(To));
        return dst;
    }

    // Largest serialized message, matches MAX_CAN_MESSAGE_SIZE in Rust.
    static constexpr size_t MAX_CAN_MESSAGE_SIZE = 64;

    // Log frames are a raw byte stream, not an encoded message. The decoder
    // ignores them. Note this type value is shared with Rust's
    // PRE_UNIX_TIME_MESSAGE_TYPE (both misc + sub type 0): the sacrificial
    // empty frame sent before each UnixTime frame, which is intentionally
    // undecodable and likewise dropped.
    static constexpr uint8_t LOG_MESSAGE_TYPE = 8;

    // CAN Extended ID implementation based on Rust's packed_struct
    // 3 bits reserved
    // 3 bits priority
    // 8 bits message_type
    // 6 bits node_type
    // 12 bits node_id
    // Total 32 bits, but only 29 bits used for actual CAN ID in frames usually,
    // though here we are mapping to a u32 representation.
    // The Rust struct is marked as MSB endian.
    struct CanBusExtendedId {
        uint8_t priority; // 3 bits
        uint8_t message_type; // 8 bits
        uint8_t node_type; // 6 bits
        uint16_t node_id; // 12 bits

        static uint32_t create(uint8_t priority, uint8_t message_type, uint8_t node_type, uint16_t node_id) noexcept {
            uint32_t id = 0;
            // Reserved 3 bits (bits 29-31) are 0
            
            // Priority: 3 bits (bits 26-28)
            id |= (static_cast<uint32_t>(priority) & 0x07) << 26;
            
            // Message Type: 8 bits (bits 18-25)
            id |= (static_cast<uint32_t>(message_type) & 0xFF) << 18;
            
            // Node Type: 6 bits (bits 12-17)
            id |= (static_cast<uint32_t>(node_type) & 0x3F) << 12;
            
            // Node ID: 12 bits (bits 0-11)
            id |= (static_cast<uint32_t>(node_id) & 0xFFF);

            return id;
        }

        static CanBusExtendedId from_raw(uint32_t raw) noexcept {
            CanBusExtendedId id;
            id.priority = (raw >> 26) & 0x07;
            id.message_type = (raw >> 18) & 0xFF;
            id.node_type = (raw >> 12) & 0x3F;
            id.node_id = raw & 0xFFF;
            return id;
        }

        static uint8_t message_type_from_raw(uint32_t raw) noexcept {
            return (raw >> 18) & 0xFF;
        }
    };

    // Helper for Big Endian serialization
    inline void write_u16_be(uint8_t* buffer, uint16_t value) noexcept {
        buffer[0] = (value >> 8) & 0xFF;
        buffer[1] = value & 0xFF;
    }

    inline void write_u24_be(uint8_t* buffer, uint32_t value) noexcept {
        buffer[0] = (value >> 16) & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = value & 0xFF;
    }

    inline void write_u32_be(uint8_t* buffer, uint32_t value) noexcept {
        buffer[0] = (value >> 24) & 0xFF;
        buffer[1] = (value >> 16) & 0xFF;
        buffer[2] = (value >> 8) & 0xFF;
        buffer[3] = value & 0xFF;
    }

    inline void write_u56_be(uint8_t* buffer, uint64_t value) noexcept {
        buffer[0] = (value >> 48) & 0xFF;
        buffer[1] = (value >> 40) & 0xFF;
        buffer[2] = (value >> 32) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
        buffer[4] = (value >> 16) & 0xFF;
        buffer[5] = (value >> 8) & 0xFF;
        buffer[6] = value & 0xFF;
    }

    inline void write_u64_be(uint8_t* buffer, uint64_t value) noexcept {
        write_u32_be(buffer, (value >> 32) & 0xFFFFFFFF);
        write_u32_be(buffer + 4, value & 0xFFFFFFFF);
    }

    inline uint16_t read_u16_be(const uint8_t* buffer) noexcept {
        return (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
    }

    inline uint32_t read_u24_be(const uint8_t* buffer) noexcept {
        return (static_cast<uint32_t>(buffer[0]) << 16) |
               (static_cast<uint32_t>(buffer[1]) << 8) |
               buffer[2];
    }

    inline uint32_t read_u32_be(const uint8_t* buffer) noexcept {
        return (static_cast<uint32_t>(buffer[0]) << 24) |
               (static_cast<uint32_t>(buffer[1]) << 16) |
               (static_cast<uint32_t>(buffer[2]) << 8) |
               buffer[3];
    }

    inline uint64_t read_u56_be(const uint8_t* buffer) noexcept {
        return (static_cast<uint64_t>(buffer[0]) << 48) |
               (static_cast<uint64_t>(buffer[1]) << 40) |
               (static_cast<uint64_t>(buffer[2]) << 32) |
               (static_cast<uint64_t>(buffer[3]) << 24) |
               (static_cast<uint64_t>(buffer[4]) << 16) |
               (static_cast<uint64_t>(buffer[5]) << 8) |
               static_cast<uint64_t>(buffer[6]);
    }

    inline uint64_t read_u64_be(const uint8_t* buffer) noexcept {
        return (static_cast<uint64_t>(read_u32_be(buffer)) << 32) |
               read_u32_be(buffer + 4);
    }

    // CRC-16/IBM-3740: poly=0x1021 init=0xFFFF refin=false refout=false xorout=0x0000
    // Matches CAN_CRC in firmware-common-new/src/can_bus/sender.rs.
    inline uint16_t can_crc16(const uint8_t* data, size_t len) noexcept {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= (static_cast<uint16_t>(data[i]) << 8);
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    // Derives the 12 bit CAN node id from a device serial number (e.g. the STM32
    // UID), same as can_node_id_from_serial_number in Rust.
    inline uint16_t can_node_id_from_serial_number(const uint8_t* serial_number, size_t len) noexcept {
        return can_crc16(serial_number, len) & 0xFFF;
    }

    struct AckMessage {
        static constexpr uint32_t MESSAGE_TYPE = 66;
        // Message Type ID to be verified against JSON
        static constexpr size_t SIZE_BYTES = 4;

        uint16_t crc;
        uint16_t node_id; // 12 bits

        AckMessage(uint16_t _crc = 0, uint16_t _node_id = 0) noexcept : crc(_crc), node_id(_node_id) {}

        static constexpr uint8_t PRIORITY = 4;

        void serialize(uint8_t* buffer) const noexcept {
             std::memset(buffer, 0, SIZE_BYTES);
             write_u16_be(buffer, crc);
             uint16_t n = node_id & 0xFFF;
             buffer[2] = (n >> 4) & 0xFF;
             buffer[3] = (n << 4) & 0xF0;
        }

        static AckMessage deserialize(const uint8_t* buffer) noexcept {
            AckMessage msg;
            msg.crc = read_u16_be(buffer);
            uint16_t n = (static_cast<uint16_t>(buffer[2]) << 4) | (buffer[3] >> 4);
            msg.node_id = n;
            return msg;
        }
    };

    enum class PowerOutputOverwrite : uint8_t {
        NoOverwrite = 0,
        ForceEnabled = 1,
        ForceDisabled = 2
    };

    struct AmpOverwriteMessage {
        static constexpr uint32_t MESSAGE_TYPE = 67;
        static constexpr size_t SIZE_BYTES = 1;

        // AMP has three power outputs; bits 6..8 are unused padding.
        PowerOutputOverwrite out1;
        PowerOutputOverwrite out2;
        PowerOutputOverwrite out3;

        AmpOverwriteMessage(PowerOutputOverwrite o1 = PowerOutputOverwrite::NoOverwrite,
                            PowerOutputOverwrite o2 = PowerOutputOverwrite::NoOverwrite,
                            PowerOutputOverwrite o3 = PowerOutputOverwrite::NoOverwrite) noexcept
            : out1(o1), out2(o2), out3(o3) {}

        static constexpr uint8_t PRIORITY = 2;

        void serialize(uint8_t* buffer) const noexcept {
            uint8_t b = 0;
            b |= (static_cast<uint8_t>(out1) & 0x03) << 6;
            b |= (static_cast<uint8_t>(out2) & 0x03) << 4;
            b |= (static_cast<uint8_t>(out3) & 0x03) << 2;
            buffer[0] = b;
        }

        static AmpOverwriteMessage deserialize(const uint8_t* buffer) noexcept {
            AmpOverwriteMessage msg;
            uint8_t b = buffer[0];
            msg.out1 = static_cast<PowerOutputOverwrite>((b >> 6) & 0x03);
            msg.out2 = static_cast<PowerOutputOverwrite>((b >> 4) & 0x03);
            msg.out3 = static_cast<PowerOutputOverwrite>((b >> 2) & 0x03);
            return msg;
        }
    };

    struct AmpResetOutputMessage {
        static constexpr uint32_t MESSAGE_TYPE = 68;
        static constexpr size_t SIZE_BYTES = 1;
        uint8_t output;

        AmpResetOutputMessage(uint8_t out = 0) noexcept : output(out) {}

        static constexpr uint8_t PRIORITY = 2;

        void serialize(uint8_t* buffer) const noexcept {
            buffer[0] = output;
        }

        static AmpResetOutputMessage deserialize(const uint8_t* buffer) noexcept {
            return AmpResetOutputMessage(buffer[0]);
        }
    };

    enum class PowerOutputStatus : uint8_t {
        Disabled = 0,
        PowerGood = 1,
        PowerBad = 2
    };

    struct AmpOutputStatus {
        bool overwrote;
        PowerOutputStatus status;

        static AmpOutputStatus from_byte(uint8_t b) noexcept {
            AmpOutputStatus s;
            s.overwrote = (b & 0x80) != 0;
            s.status = static_cast<PowerOutputStatus>((b >> 5) & 0x03);
            return s;
        }

        uint8_t to_byte() const noexcept {
            uint8_t b = 0;
            if (overwrote) b |= 0x80;
            b |= (static_cast<uint8_t>(status) & 0x03) << 5;
            return b;
        }
    };

    struct AmpStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 33;
        static constexpr size_t SIZE_BYTES = 5;
        
        uint16_t shared_battery_mv;
        AmpOutputStatus out1;
        AmpOutputStatus out2;
        AmpOutputStatus out3;

        static constexpr uint8_t PRIORITY = 5;

        void serialize(uint8_t* buffer) const noexcept {
            std::memset(buffer, 0, SIZE_BYTES);
            write_u16_be(buffer, shared_battery_mv);
            buffer[2] = out1.to_byte();
            buffer[3] = out2.to_byte();
            buffer[4] = out3.to_byte();
        }

        static AmpStatusMessage deserialize(const uint8_t* buffer) noexcept {
            AmpStatusMessage msg;
            msg.shared_battery_mv = read_u16_be(buffer);
            msg.out1 = AmpOutputStatus::from_byte(buffer[2]);
            msg.out2 = AmpOutputStatus::from_byte(buffer[3]);
            msg.out3 = AmpOutputStatus::from_byte(buffer[4]);
            return msg;
        }
    };

    struct BaroMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 128;
        static constexpr size_t SIZE_BYTES = 13;

        uint32_t pressure_raw;
        uint16_t temperature_raw;
        uint64_t timestamp_us;

        // Helpers
        static BaroMeasurementMessage new_msg(uint64_t ts, float pressure, float temperature) noexcept {
            BaroMeasurementMessage msg;
            msg.timestamp_us = ts;
            msg.pressure_raw = bit_cast<uint32_t>(pressure);
            msg.temperature_raw = static_cast<uint16_t>(temperature * 10.0f);
            return msg;
        }

        float pressure() const noexcept {
            return bit_cast<float>(pressure_raw);
        }

        float temperature() const noexcept {
            return static_cast<float>(temperature_raw) / 10.0f;
        }

        static constexpr uint8_t PRIORITY = 3;

        void serialize(uint8_t* buffer) const noexcept {
             write_u32_be(buffer, pressure_raw);
             write_u16_be(buffer + 4, temperature_raw);
             write_u56_be(buffer + 6, timestamp_us);
        }

        static BaroMeasurementMessage deserialize(const uint8_t* buffer) noexcept {
            BaroMeasurementMessage msg;
            msg.pressure_raw = read_u32_be(buffer);
            msg.temperature_raw = read_u16_be(buffer + 4);
            msg.timestamp_us = read_u56_be(buffer + 6);
            return msg;
        }
    };

    struct BrightnessMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 130;
        static constexpr size_t SIZE_BYTES = 11;

        uint32_t brightness_lux_raw;
        uint64_t timestamp_us;

        static BrightnessMeasurementMessage new_msg(uint64_t ts, float lux) noexcept {
             BrightnessMeasurementMessage msg;
             msg.timestamp_us = ts;
             msg.brightness_lux_raw = bit_cast<uint32_t>(lux);
             return msg;
        }

        float brightness_lux() const noexcept {
            return bit_cast<float>(brightness_lux_raw);
        }

        static constexpr uint8_t PRIORITY = 5;

        void serialize(uint8_t* buffer) const noexcept {
            write_u32_be(buffer, brightness_lux_raw);
            write_u56_be(buffer + 4, timestamp_us);
        }

        static BrightnessMeasurementMessage deserialize(const uint8_t* buffer) noexcept {
            BrightnessMeasurementMessage msg;
            msg.brightness_lux_raw = read_u32_be(buffer);
            msg.timestamp_us = read_u56_be(buffer + 4);
            return msg;
        }
    };

    // Extended EPM telemetry from the payload SDRM node, sent every 500ms.
    //
    // Supplementary to NodeStatusMessage, which stays the primary go/no-go source.
    // Deliberately does not repeat uptime_s, health, mode or the stack flags, so
    // the two messages can not drift apart.
    //
    // Voltages are relayed from EPM on the intra-stack bus, not measured by SDRM.
    struct CustomPayloadStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 35;
        static constexpr size_t SIZE_BYTES = 10;

        // Reported for a rail whose reading is invalid or unavailable.
        static constexpr uint16_t RAIL_MV_UNAVAILABLE = 0xFFFF;

        uint16_t epm_batt_mv;
        uint16_t epm_sys_3v3_mv;
        uint16_t epm_sys_5v_mv;
        uint16_t epm_per_5v_mv;
        uint16_t epm_per_9v_mv;

        static constexpr uint8_t PRIORITY = 5;

        // Every rail unavailable, e.g. before EPM has reported.
        static CustomPayloadStatusMessage new_unavailable() noexcept {
            CustomPayloadStatusMessage msg;
            msg.epm_batt_mv = RAIL_MV_UNAVAILABLE;
            msg.epm_sys_3v3_mv = RAIL_MV_UNAVAILABLE;
            msg.epm_sys_5v_mv = RAIL_MV_UNAVAILABLE;
            msg.epm_per_5v_mv = RAIL_MV_UNAVAILABLE;
            msg.epm_per_9v_mv = RAIL_MV_UNAVAILABLE;
            return msg;
        }

        // std::nullopt if the reading is invalid or unavailable.
        static std::optional<uint16_t> rail_mv(uint16_t raw_mv) noexcept {
            if (raw_mv == RAIL_MV_UNAVAILABLE) return std::nullopt;
            return raw_mv;
        }

        void serialize(uint8_t* buffer) const noexcept {
            write_u16_be(buffer, epm_batt_mv);
            write_u16_be(buffer + 2, epm_sys_3v3_mv);
            write_u16_be(buffer + 4, epm_sys_5v_mv);
            write_u16_be(buffer + 6, epm_per_5v_mv);
            write_u16_be(buffer + 8, epm_per_9v_mv);
        }

        static CustomPayloadStatusMessage deserialize(const uint8_t* buffer) noexcept {
            CustomPayloadStatusMessage msg;
            msg.epm_batt_mv = read_u16_be(buffer);
            msg.epm_sys_3v3_mv = read_u16_be(buffer + 2);
            msg.epm_sys_5v_mv = read_u16_be(buffer + 4);
            msg.epm_per_5v_mv = read_u16_be(buffer + 6);
            msg.epm_per_9v_mv = read_u16_be(buffer + 8);
            return msg;
        }
    };

    enum class DataType : uint8_t {
        Data = 1
    };

    struct DataTransferMessage {
        static constexpr uint32_t MESSAGE_TYPE = 16;
        static constexpr size_t SIZE_BYTES = 36;
        static constexpr size_t DATA_CAPACITY = 32;

        uint8_t data[DATA_CAPACITY];
        uint8_t data_len;
        uint8_t sequence_number;
        bool start_of_transfer;
        bool end_of_transfer;
        DataType data_type;
        uint16_t destination_node_id; // 12 bits

        DataTransferMessage() noexcept : data{0}, data_len(0), sequence_number(0), 
            start_of_transfer(false), end_of_transfer(false), data_type(DataType::Data), destination_node_id(0) {}

        static constexpr uint8_t PRIORITY = 6;

        void serialize(uint8_t* buffer) const noexcept {
            // 0..32: data
            std::memcpy(buffer, data, DATA_CAPACITY);
            // 32: data_len
            buffer[32] = data_len;
            // 33: sequence_number
            buffer[33] = sequence_number;
            
            // 34, 35: packed fields
            uint8_t b34 = 0;
            if (start_of_transfer) b34 |= 0x80;
            if (end_of_transfer) b34 |= 0x40;
            b34 |= (static_cast<uint8_t>(data_type) & 0x03) << 4;
            
            // destination_node_id 12 bits. Top 4 bits to b34 lower nibble
            b34 |= (destination_node_id >> 8) & 0x0F;
            buffer[34] = b34;
            
            // Bottom 8 bits to b35
            buffer[35] = destination_node_id & 0xFF;
        }

        static DataTransferMessage deserialize(const uint8_t* buffer) noexcept {
            DataTransferMessage msg;
            std::memcpy(msg.data, buffer, DATA_CAPACITY);
            msg.data_len = buffer[32];
            msg.sequence_number = buffer[33];
            
            uint8_t b34 = buffer[34];
            msg.start_of_transfer = (b34 & 0x80) != 0;
            msg.end_of_transfer = (b34 & 0x40) != 0;
            msg.data_type = static_cast<DataType>((b34 >> 4) & 0x03);
            
            msg.destination_node_id = ((static_cast<uint16_t>(b34) & 0x0F) << 8) | buffer[35];
            return msg;
        }
    };

    struct IcarusStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 160;
        static constexpr size_t SIZE_BYTES = 6;

        uint16_t actual_extension_percentage; // 0.1%
        uint16_t servo_temperature_raw; // 0.1C
        uint16_t servo_current_raw; // 0.01A

        static IcarusStatusMessage new_msg(float extension, float temp, float current) noexcept {
            IcarusStatusMessage msg;
            msg.actual_extension_percentage = static_cast<uint16_t>(extension * 1000.0f);
            msg.servo_temperature_raw = static_cast<uint16_t>(temp * 10.0f);
            msg.servo_current_raw = static_cast<uint16_t>(current * 100.0f);
            return msg;
        }

        float actual_extension_percentage_float() const noexcept {
            return static_cast<float>(actual_extension_percentage) / 1000.0f;
        }
        float servo_temperature() const noexcept {
            return static_cast<float>(servo_temperature_raw) / 10.0f;
        }
        float servo_current() const noexcept {
            return static_cast<float>(servo_current_raw) / 100.0f;
        }

        static constexpr uint8_t PRIORITY = 5;

        void serialize(uint8_t* buffer) const noexcept {
            write_u16_be(buffer, actual_extension_percentage);
            write_u16_be(buffer + 2, servo_temperature_raw);
            write_u16_be(buffer + 4, servo_current_raw);
        }

        static IcarusStatusMessage deserialize(const uint8_t* buffer) noexcept {
            IcarusStatusMessage msg;
            msg.actual_extension_percentage = read_u16_be(buffer);
            msg.servo_temperature_raw = read_u16_be(buffer + 2);
            msg.servo_current_raw = read_u16_be(buffer + 4);
            return msg;
        }
    };

    struct IMUMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 129;
        static constexpr size_t SIZE_BYTES = 31;
        
        uint32_t acc_raw[3];
        uint32_t gyro_raw[3];
        uint64_t timestamp_us;

        static IMUMeasurementMessage new_msg(uint64_t ts, const float acc[3], const float gyro[3]) noexcept {
            IMUMeasurementMessage msg;
            msg.timestamp_us = ts;
            for(int i=0; i<3; i++) {
                msg.acc_raw[i] = bit_cast<uint32_t>(acc[i]);
                msg.gyro_raw[i] = bit_cast<uint32_t>(gyro[i]);
            }
            return msg;
        }

        void acc(float out[3]) const noexcept {
             for(int i=0; i<3; i++) {
                 out[i] = bit_cast<float>(acc_raw[i]);
             }
        }

        void gyro(float out[3]) const noexcept {
             for(int i=0; i<3; i++) {
                 out[i] = bit_cast<float>(gyro_raw[i]);
             }
        }

        static constexpr uint8_t PRIORITY = 3;

        void serialize(uint8_t* buffer) const noexcept {
            for(int i=0; i<3; i++) write_u32_be(buffer + i*4, acc_raw[i]);
            for(int i=0; i<3; i++) write_u32_be(buffer + 12 + i*4, gyro_raw[i]);
            write_u56_be(buffer + 24, timestamp_us);
        }

        static IMUMeasurementMessage deserialize(const uint8_t* buffer) noexcept {
            IMUMeasurementMessage msg;
            for(int i=0; i<3; i++) msg.acc_raw[i] = read_u32_be(buffer + i*4);
            for(int i=0; i<3; i++) msg.gyro_raw[i] = read_u32_be(buffer + 12 + i*4);
            msg.timestamp_us = read_u56_be(buffer + 24);
            return msg;
        }
    };

    struct MagMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 132;
        static constexpr size_t SIZE_BYTES = 19;
        
        uint32_t mag_raw[3];
        uint64_t timestamp_us;

        static MagMeasurementMessage new_msg(uint64_t ts, const float mag[3]) noexcept {
            MagMeasurementMessage msg;
            msg.timestamp_us = ts;
            for(int i=0; i<3; i++) {
                msg.mag_raw[i] = bit_cast<uint32_t>(mag[i]);
            }
            return msg;
        }

        void mag(float out[3]) const noexcept {
             for(int i=0; i<3; i++) {
                 out[i] = bit_cast<float>(mag_raw[i]);
             }
        }

        static constexpr uint8_t PRIORITY = 3;

        void serialize(uint8_t* buffer) const noexcept {
            for(int i=0; i<3; i++) write_u32_be(buffer + i*4, mag_raw[i]);
            write_u56_be(buffer + 12, timestamp_us);
        }

        static MagMeasurementMessage deserialize(const uint8_t* buffer) noexcept {
            MagMeasurementMessage msg;
            for(int i=0; i<3; i++) msg.mag_raw[i] = read_u32_be(buffer + i*4);
            msg.timestamp_us = read_u56_be(buffer + 12);
            return msg;
        }
    };

    enum class NodeHealth : uint8_t {
        Healthy = 0,
        Warning = 1,
        Error = 2,
        Critical = 3
    };

    enum class NodeMode : uint8_t {
        Operational = 0,
        Initialization = 1,
        Maintenance = 2,
        Offline = 3
    };

    struct NodeStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 32;
        static constexpr size_t SIZE_BYTES = 5;

        uint32_t uptime_s; // 24 bits
        NodeHealth health; // 2 bits
        NodeMode mode;     // 2 bits
        uint16_t custom_status_raw; // 11 bits

        NodeStatusMessage(uint32_t _uptime_s = 0, NodeHealth _health = NodeHealth::Healthy, NodeMode _mode = NodeMode::Operational, uint16_t _custom_status_raw = 0) noexcept
            : uptime_s(_uptime_s), health(_health), mode(_mode), custom_status_raw(_custom_status_raw) {}

        static constexpr uint8_t PRIORITY = 5;

        void serialize(uint8_t* buffer) const noexcept {
            write_u24_be(buffer, uptime_s);
            
            // Byte 3: health(2), mode(2), custom_status_raw(4 top bits)
            uint8_t b3 = 0;
            b3 |= (static_cast<uint8_t>(health) & 0x03) << 6;
            b3 |= (static_cast<uint8_t>(mode) & 0x03) << 4;
            // custom_status_raw is 11 bits. Top 4 bits: 10..7
            b3 |= (custom_status_raw >> 7) & 0x0F;
            buffer[3] = b3;
            
            // Byte 4: custom_status_raw(7 bottom bits)
            buffer[4] = (custom_status_raw & 0x7F) << 1; 
        }

        static NodeStatusMessage deserialize(const uint8_t* buffer) noexcept {
            NodeStatusMessage msg;
            msg.uptime_s = read_u24_be(buffer);
            
            uint8_t b3 = buffer[3];
            msg.health = static_cast<NodeHealth>((b3 >> 6) & 0x03);
            msg.mode = static_cast<NodeMode>((b3 >> 4) & 0x03);
            
            uint16_t csr = (b3 & 0x0F) << 7;
            csr |= (buffer[4] >> 1) & 0x7F;
            msg.custom_status_raw = csr;
            return msg;
        }
    };

    // Stack state of the payload SDRM node, packed into
    // NodeStatusMessage::custom_status_raw.
    //
    // Uses all 11 available bits, declaration order most-significant first:
    // epm_alive occupies bit 10 and fault occupies bit 0.
    //
    //     PayloadSDRMCustomStatus status;
    //     status.epm_alive = true;
    //     status.stack_powered = true;
    //     NodeStatusMessage msg(uptime_s, NodeHealth::Healthy,
    //                          NodeMode::Operational, status.to_raw());
    struct PayloadSDRMCustomStatus {
        bool epm_alive = false;             // EPM responded on the intra-stack bus
        bool sem_alive = false;             // SEM responded on the intra-stack bus
        bool stack_powered = false;         // power_on complete
        bool sdrm_sd_logging = false;       // SDRM SD log active
        bool sem_sd_logging = false;        // SEM SD log active
        bool exp1_active = false;           // Experiment channel 1 active
        bool exp2_active = false;           // Experiment channel 2 active
        bool exp3_active = false;           // Experiment channel 3 active
        bool prep_complete = false;         // Tare + home complete for channels 1..3
        bool armed_bundle_complete = false; // Full Armed sequence finished OK
        bool fault = false;                 // Last stack action failed

        // Pack into the 11-bit value carried by NodeStatusMessage::custom_status_raw.
        uint16_t to_raw() const noexcept {
            uint16_t raw = 0;
            if (epm_alive)             raw |= 1u << 10;
            if (sem_alive)             raw |= 1u << 9;
            if (stack_powered)         raw |= 1u << 8;
            if (sdrm_sd_logging)       raw |= 1u << 7;
            if (sem_sd_logging)        raw |= 1u << 6;
            if (exp1_active)           raw |= 1u << 5;
            if (exp2_active)           raw |= 1u << 4;
            if (exp3_active)           raw |= 1u << 3;
            if (prep_complete)         raw |= 1u << 2;
            if (armed_bundle_complete) raw |= 1u << 1;
            if (fault)                 raw |= 1u << 0;
            return raw;
        }

        static PayloadSDRMCustomStatus from_raw(uint16_t raw) noexcept {
            PayloadSDRMCustomStatus status;
            status.epm_alive             = (raw & (1u << 10)) != 0;
            status.sem_alive             = (raw & (1u << 9)) != 0;
            status.stack_powered         = (raw & (1u << 8)) != 0;
            status.sdrm_sd_logging       = (raw & (1u << 7)) != 0;
            status.sem_sd_logging        = (raw & (1u << 6)) != 0;
            status.exp1_active           = (raw & (1u << 5)) != 0;
            status.exp2_active           = (raw & (1u << 4)) != 0;
            status.exp3_active           = (raw & (1u << 3)) != 0;
            status.prep_complete         = (raw & (1u << 2)) != 0;
            status.armed_bundle_complete = (raw & (1u << 1)) != 0;
            status.fault                 = (raw & (1u << 0)) != 0;
            return status;
        }

        // Clears expN_active, prep_complete and armed_bundle_complete, leaving
        // liveness, power, logging and fault untouched.
        //
        // Applied after the LowPower safe-reset completes.
        void clear_experiment_flags() noexcept {
            exp1_active = false;
            exp2_active = false;
            exp3_active = false;
            prep_complete = false;
            armed_bundle_complete = false;
        }

        // Clears clear_experiment_flags() plus stack_powered and both SD logging
        // flags, leaving liveness and fault untouched.
        //
        // Applied after the Landed shutdown completes.
        void clear_powered_flags() noexcept {
            clear_experiment_flags();
            stack_powered = false;
            sdrm_sd_logging = false;
            sem_sd_logging = false;
        }
    };

    struct OzysMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 133;
        static constexpr size_t SIZE_BYTES = 16;
        
        uint32_t sg_1_raw;
        uint32_t sg_2_raw;
        uint32_t sg_3_raw;
        uint32_t sg_4_raw;

        // An absent channel is carried as NaN, same as the Rust Option<f32>.
        static OzysMeasurementMessage new_msg(std::optional<float> sg_1, std::optional<float> sg_2,
                                              std::optional<float> sg_3, std::optional<float> sg_4) noexcept {
            OzysMeasurementMessage msg;
            msg.sg_1_raw = bit_cast<uint32_t>(sg_1.value_or(NAN));
            msg.sg_2_raw = bit_cast<uint32_t>(sg_2.value_or(NAN));
            msg.sg_3_raw = bit_cast<uint32_t>(sg_3.value_or(NAN));
            msg.sg_4_raw = bit_cast<uint32_t>(sg_4.value_or(NAN));
            return msg;
        }

        static std::optional<float> sg(uint32_t raw) noexcept {
            float v = bit_cast<float>(raw);
            if (std::isnan(v)) return std::nullopt;
            return v;
        }

        std::optional<float> sg_1() const noexcept { return sg(sg_1_raw); }
        std::optional<float> sg_2() const noexcept { return sg(sg_2_raw); }
        std::optional<float> sg_3() const noexcept { return sg(sg_3_raw); }
        std::optional<float> sg_4() const noexcept { return sg(sg_4_raw); }

        static constexpr uint8_t PRIORITY = 5;

        void serialize(uint8_t* buffer) const noexcept {
            write_u32_be(buffer, sg_1_raw);
            write_u32_be(buffer + 4, sg_2_raw);
            write_u32_be(buffer + 8, sg_3_raw);
            write_u32_be(buffer + 12, sg_4_raw);
        }

        static OzysMeasurementMessage deserialize(const uint8_t* buffer) noexcept {
            OzysMeasurementMessage msg;
            msg.sg_1_raw = read_u32_be(buffer);
            msg.sg_2_raw = read_u32_be(buffer + 4);
            msg.sg_3_raw = read_u32_be(buffer + 8);
            msg.sg_4_raw = read_u32_be(buffer + 12);
            return msg;
        }
    };

    struct ResetMessage {
        static constexpr uint32_t MESSAGE_TYPE = 0;
        static constexpr size_t SIZE_BYTES = 2;

        uint16_t node_id; // 12 bits
        bool reset_all;

        static constexpr uint8_t PRIORITY = 0;

        void serialize(uint8_t* buffer) const noexcept {
            // Byte 0: node_id[11..4]
            buffer[0] = (node_id >> 4) & 0xFF;

            // Byte 1: node_id[3..0], reset_all, padding(3)
            uint8_t b1 = 0;
            b1 |= (node_id & 0x0F) << 4;
            if (reset_all) b1 |= 0x08;
            buffer[1] = b1;
        }

        static ResetMessage deserialize(const uint8_t* buffer) noexcept {
            ResetMessage msg;
            msg.node_id = (static_cast<uint16_t>(buffer[0]) << 4) | ((buffer[1] >> 4) & 0x0F);
            msg.reset_all = (buffer[1] & 0x08) != 0;
            return msg;
        }
    };

    struct RocketStateMessage {
        static constexpr uint32_t MESSAGE_TYPE = 131;
        static constexpr size_t SIZE_BYTES = 19;

        uint32_t velocity_raw[2];
        uint32_t altitude_agl_raw;
        uint64_t timestamp_us;

        static RocketStateMessage new_msg(uint64_t ts, const float vel[2], float altitude_agl) noexcept {
            RocketStateMessage msg;
            msg.timestamp_us = ts;
            for(int i=0; i<2; i++) {
                msg.velocity_raw[i] = bit_cast<uint32_t>(vel[i]);
            }
            msg.altitude_agl_raw = bit_cast<uint32_t>(altitude_agl);
            return msg;
        }

        void velocity(float out[2]) const noexcept {
             for(int i=0; i<2; i++) {
                 out[i] = bit_cast<float>(velocity_raw[i]);
             }
        }

        float altitude_agl() const noexcept {
            return bit_cast<float>(altitude_agl_raw);
        }

        static constexpr uint8_t PRIORITY = 3;

        void serialize(uint8_t* buffer) const noexcept {
            write_u32_be(buffer, velocity_raw[0]);
            write_u32_be(buffer + 4, velocity_raw[1]);
            write_u32_be(buffer + 8, altitude_agl_raw);
            write_u56_be(buffer + 12, timestamp_us);
        }

        static RocketStateMessage deserialize(const uint8_t* buffer) noexcept {
            RocketStateMessage msg;
            msg.velocity_raw[0] = read_u32_be(buffer);
            msg.velocity_raw[1] = read_u32_be(buffer + 4);
            msg.altitude_agl_raw = read_u32_be(buffer + 8);
            msg.timestamp_us = read_u56_be(buffer + 12);
            return msg;
        }
    };

    struct UnixTimeMessage {
        static constexpr uint32_t MESSAGE_TYPE = 7;
        static constexpr size_t SIZE_BYTES = 7;
        uint64_t timestamp_us;

        static constexpr uint8_t PRIORITY = 1;

        void serialize(uint8_t* buffer) const noexcept {
            write_u56_be(buffer, timestamp_us);
        }

        static UnixTimeMessage deserialize(const uint8_t* buffer) noexcept {
            UnixTimeMessage msg;
            msg.timestamp_us = read_u56_be(buffer);
            return msg;
        }
    };

    enum class FlightStage : uint8_t {
        LowPower = 0,
        SelfTest = 1,
        Armed = 2,
        Ascent = 3,
        MachLockout = 4,
        DrogueChute = 5,
        MainChute = 6,
        Landed = 7,
        FailedToReachMinApogee = 8
    };

    struct VLStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 36;
        static constexpr size_t SIZE_BYTES = 5;

        FlightStage flight_stage;
        uint16_t battery_mv;

        static constexpr uint8_t PRIORITY = 2;

        void serialize(uint8_t* buffer) const noexcept {
            buffer[0] = static_cast<uint8_t>(flight_stage);
            write_u16_be(buffer + 1, battery_mv);
            buffer[3] = 0;
            buffer[4] = 0;
        }

        static VLStatusMessage deserialize(const uint8_t* buffer) noexcept {
            VLStatusMessage msg;
            msg.flight_stage = static_cast<FlightStage>(buffer[0]);
            msg.battery_mv = read_u16_be(buffer + 1);
            return msg;
        }
    };


    struct AirBrakesControlMessage {
        static constexpr uint32_t MESSAGE_TYPE = 69;
        static constexpr size_t SIZE_BYTES = 6;

        uint16_t extension_percentage; // Unit: 0.1%, e.g. 10 = 1%

        // Constructor for convenience
        AirBrakesControlMessage(uint16_t ext_pct = 0) noexcept : extension_percentage(ext_pct) {}
        
        // Helper to convert from float percentage (0.0 - 100.0)
        static AirBrakesControlMessage from_percentage(float percentage) noexcept {
            return AirBrakesControlMessage(static_cast<uint16_t>(percentage * 10.0f));
        }
        
        // Rust 'new' equivalent with float input (0.0 - 1.0 range based on Rust code)
        static AirBrakesControlMessage from_float(float percentage) noexcept {
            return AirBrakesControlMessage(static_cast<uint16_t>(percentage * 1000.0f));
        }

        float to_float() const noexcept {
             return static_cast<float>(extension_percentage) / 1000.0f;
        }

        static constexpr uint8_t PRIORITY = 2;

        void serialize(uint8_t* buffer) const noexcept {
            std::memset(buffer, 0, SIZE_BYTES); // Zero out for padding
            write_u16_be(buffer, extension_percentage);
        }

        static AirBrakesControlMessage deserialize(const uint8_t* buffer) noexcept {
            AirBrakesControlMessage msg;
            msg.extension_percentage = read_u16_be(buffer);
            return msg;
        }
    };

    struct AmpControlMessage {
        static constexpr uint32_t MESSAGE_TYPE = 64;
        static constexpr size_t SIZE_BYTES = 1;

        // AMP has three power outputs; bits 3..8 are unused padding.
        bool out1_enable;
        bool out2_enable;
        bool out3_enable;

        AmpControlMessage(bool o1 = false, bool o2 = false, bool o3 = false) noexcept
            : out1_enable(o1), out2_enable(o2), out3_enable(o3) {}

        static constexpr uint8_t PRIORITY = 2;

        void serialize(uint8_t* buffer) const noexcept {
            uint8_t byte = 0;
            if (out1_enable) byte |= (1 << 7); // MSB0 bit 0 -> 7 in LSB
            if (out2_enable) byte |= (1 << 6);
            if (out3_enable) byte |= (1 << 5);
            // Remaining bits are 0
            buffer[0] = byte;
        }

        static AmpControlMessage deserialize(const uint8_t* buffer) noexcept {
            AmpControlMessage msg;
            uint8_t byte = buffer[0];
            msg.out1_enable = (byte & (1 << 7)) != 0;
            msg.out2_enable = (byte & (1 << 6)) != 0;
            msg.out3_enable = (byte & (1 << 5)) != 0;
            return msg;
        }
    };


    struct TailByte {
        bool start_of_transfer;
        bool end_of_transfer;
        bool toggle;

        TailByte(bool s = false, bool e = false, bool t = false) noexcept
            : start_of_transfer(s), end_of_transfer(e), toggle(t) {}

        uint8_t to_byte() const noexcept {
            uint8_t b = 0;
            if (start_of_transfer) b |= 0x80;
            if (end_of_transfer) b |= 0x40;
            if (toggle) b |= 0x20;
            return b;
        }

        static TailByte from_byte(uint8_t b) noexcept {
            return TailByte((b & 0x80) != 0, (b & 0x40) != 0, (b & 0x20) != 0);
        }
    };

    // Returns a mask for the CAN hardware's acceptance filter, mirroring
    // create_can_bus_message_type_filter_mask in Rust. Declared here rather than
    // next to CanBusExtendedId because it needs the message type constants.
    //
    // Filter logic: `frame_accepted = (incoming_id & mask) == 0`
    //
    // - a frame whose message type is in `accept_message_types` is always accepted
    // - a frame whose message type is not in the list *MAY OR MAY NOT* be rejected,
    //   so the receiver still has to check the type itself
    // - ResetMessage and UnixTimeMessage are always accepted, even when absent
    //   from the list
    inline uint32_t create_can_bus_message_type_filter_mask(const uint8_t* accept_message_types, size_t count) noexcept {
        uint8_t accept_message_type_ored = 0;
        for (size_t i = 0; i < count; ++i) {
            accept_message_type_ored |= accept_message_types[i];
        }
        accept_message_type_ored |= static_cast<uint8_t>(ResetMessage::MESSAGE_TYPE);
        accept_message_type_ored |= static_cast<uint8_t>(UnixTimeMessage::MESSAGE_TYPE);

        return CanBusExtendedId::create(0, static_cast<uint8_t>(~accept_message_type_ored), 0, 0);
    }

    inline uint32_t create_can_bus_message_type_filter_mask(std::initializer_list<uint8_t> accept_message_types) noexcept {
        return create_can_bus_message_type_filter_mask(accept_message_types.begin(), accept_message_types.size());
    }

    using CanBusMessage = std::variant<
        std::monostate, // Represents no message or error
        AckMessage,
        AirBrakesControlMessage,
        AmpControlMessage,
        AmpOverwriteMessage,
        AmpResetOutputMessage,
        AmpStatusMessage,
        BaroMeasurementMessage,
        BrightnessMeasurementMessage,
        CustomPayloadStatusMessage,
        DataTransferMessage,
        IcarusStatusMessage,
        IMUMeasurementMessage,
        MagMeasurementMessage,
        NodeStatusMessage,
        OzysMeasurementMessage,
        ResetMessage,
        RocketStateMessage,
        UnixTimeMessage,
        VLStatusMessage
    >;

    inline uint32_t get_frame_id(const CanBusMessage& message, uint8_t node_type, uint16_t node_id) noexcept {
        return std::visit([node_type, node_id](const auto& msg) -> uint32_t {
            using T = std::decay_t<decltype(msg)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
                return 0;
            } else {
                return CanBusExtendedId::create(T::PRIORITY, T::MESSAGE_TYPE, node_type, node_id);
            }
        }, message);
    }

    class CanBusMultiFrameEncoder {
    public:
        CanBusMultiFrameEncoder(const CanBusMessage& message) noexcept
            : offset(0), toggle(false) {
            std::visit([this](const auto& msg) {
                using T = std::decay_t<decltype(msg)>;
                if constexpr (std::is_same_v<T, std::monostate>) {
                    this->message_len = 0;
                } else {
                    this->message_len = T::SIZE_BYTES;
                    msg.serialize(this->serialized_message);
                }
            }, message);
            this->crc = calculate_crc(this->serialized_message, this->message_len);
        }

        struct Frame {
            uint8_t data[8];
            size_t len;
        };

        bool has_next() const noexcept {
            return offset < message_len;
        }

        Frame next() noexcept {
            Frame frame;
            std::memset(frame.data, 0, 8);

            if (offset == 0 && message_len <= 7) {
                // Single frame message
                std::memcpy(frame.data, serialized_message, message_len);
                frame.data[message_len] = TailByte(true, true, false).to_byte();
                frame.len = message_len + 1;
                offset = message_len;
            } else {
                // Multi-frame message
                if (offset == 0) {
                    // First frame
                    frame.data[0] = crc & 0xFF;
                    frame.data[1] = (crc >> 8) & 0xFF;
                    std::memcpy(frame.data + 2, serialized_message, 5);
                    frame.data[7] = TailByte(true, false, toggle).to_byte();
                    frame.len = 8;
                    offset = 5;
                } else if (offset + 7 >= message_len) {
                    // Last frame
                    size_t remaining = message_len - offset;
                    std::memcpy(frame.data, serialized_message + offset, remaining);
                    frame.data[remaining] = TailByte(false, true, toggle).to_byte();
                    frame.len = remaining + 1;
                    offset = message_len;
                } else {
                    // Middle frame
                    std::memcpy(frame.data, serialized_message + offset, 7);
                    frame.data[7] = TailByte(false, false, toggle).to_byte();
                    frame.len = 8;
                    offset += 7;
                }
                toggle = !toggle;
            }
            return frame;
        }

        uint16_t get_crc() const noexcept { return crc; }

    private:
        uint8_t serialized_message[MAX_CAN_MESSAGE_SIZE];
        size_t message_len;
        size_t offset;
        bool toggle;
        uint16_t crc;

        static uint16_t calculate_crc(const uint8_t* data, size_t len) noexcept {
            return can_crc16(data, len);
        }
    };

    // Serialized length of a message type, std::nullopt if the type is unknown.
    inline std::optional<size_t> serialized_len(uint8_t message_type) noexcept {
        switch(message_type) {
            case AckMessage::MESSAGE_TYPE: return AckMessage::SIZE_BYTES;
            case AirBrakesControlMessage::MESSAGE_TYPE: return AirBrakesControlMessage::SIZE_BYTES;
            case AmpControlMessage::MESSAGE_TYPE: return AmpControlMessage::SIZE_BYTES;
            case AmpOverwriteMessage::MESSAGE_TYPE: return AmpOverwriteMessage::SIZE_BYTES;
            case AmpResetOutputMessage::MESSAGE_TYPE: return AmpResetOutputMessage::SIZE_BYTES;
            case AmpStatusMessage::MESSAGE_TYPE: return AmpStatusMessage::SIZE_BYTES;
            case BaroMeasurementMessage::MESSAGE_TYPE: return BaroMeasurementMessage::SIZE_BYTES;
            case BrightnessMeasurementMessage::MESSAGE_TYPE: return BrightnessMeasurementMessage::SIZE_BYTES;
            case CustomPayloadStatusMessage::MESSAGE_TYPE: return CustomPayloadStatusMessage::SIZE_BYTES;
            case DataTransferMessage::MESSAGE_TYPE: return DataTransferMessage::SIZE_BYTES;
            case IcarusStatusMessage::MESSAGE_TYPE: return IcarusStatusMessage::SIZE_BYTES;
            case IMUMeasurementMessage::MESSAGE_TYPE: return IMUMeasurementMessage::SIZE_BYTES;
            case MagMeasurementMessage::MESSAGE_TYPE: return MagMeasurementMessage::SIZE_BYTES;
            case NodeStatusMessage::MESSAGE_TYPE: return NodeStatusMessage::SIZE_BYTES;
            case OzysMeasurementMessage::MESSAGE_TYPE: return OzysMeasurementMessage::SIZE_BYTES;
            case ResetMessage::MESSAGE_TYPE: return ResetMessage::SIZE_BYTES;
            case RocketStateMessage::MESSAGE_TYPE: return RocketStateMessage::SIZE_BYTES;
            case UnixTimeMessage::MESSAGE_TYPE: return UnixTimeMessage::SIZE_BYTES;
            case VLStatusMessage::MESSAGE_TYPE: return VLStatusMessage::SIZE_BYTES;
            default:
                return std::nullopt;
        }
    }

    // `len` must be the exact serialized length of the message type, mirroring
    // Rust's PackedStructSlice; a short or long buffer decodes to std::nullopt
    // instead of reading out of bounds.
    inline std::optional<CanBusMessage> decode(uint8_t message_type, const uint8_t* buffer, size_t len) noexcept {
        auto expected_len = serialized_len(message_type);
        if (!expected_len) return std::nullopt;
        if (len != *expected_len) {
            return std::nullopt;
        }

        switch(message_type) {
            case AckMessage::MESSAGE_TYPE:
                return CanBusMessage(AckMessage::deserialize(buffer));
            case AirBrakesControlMessage::MESSAGE_TYPE:
                return CanBusMessage(AirBrakesControlMessage::deserialize(buffer));
            case AmpControlMessage::MESSAGE_TYPE:
                return CanBusMessage(AmpControlMessage::deserialize(buffer));
            case AmpOverwriteMessage::MESSAGE_TYPE:
                return CanBusMessage(AmpOverwriteMessage::deserialize(buffer));
            case AmpResetOutputMessage::MESSAGE_TYPE:
                return CanBusMessage(AmpResetOutputMessage::deserialize(buffer));
            case AmpStatusMessage::MESSAGE_TYPE:
                return CanBusMessage(AmpStatusMessage::deserialize(buffer));
            case BaroMeasurementMessage::MESSAGE_TYPE:
                return CanBusMessage(BaroMeasurementMessage::deserialize(buffer));
            case BrightnessMeasurementMessage::MESSAGE_TYPE:
                return CanBusMessage(BrightnessMeasurementMessage::deserialize(buffer));
            case CustomPayloadStatusMessage::MESSAGE_TYPE:
                return CanBusMessage(CustomPayloadStatusMessage::deserialize(buffer));
            case DataTransferMessage::MESSAGE_TYPE:
                return CanBusMessage(DataTransferMessage::deserialize(buffer));
            case IcarusStatusMessage::MESSAGE_TYPE: 
                return CanBusMessage(IcarusStatusMessage::deserialize(buffer));
            case IMUMeasurementMessage::MESSAGE_TYPE:
                return CanBusMessage(IMUMeasurementMessage::deserialize(buffer));
            case MagMeasurementMessage::MESSAGE_TYPE:
                return CanBusMessage(MagMeasurementMessage::deserialize(buffer));
            case NodeStatusMessage::MESSAGE_TYPE: 
                return CanBusMessage(NodeStatusMessage::deserialize(buffer));
            case OzysMeasurementMessage::MESSAGE_TYPE:
                return CanBusMessage(OzysMeasurementMessage::deserialize(buffer));
            case ResetMessage::MESSAGE_TYPE:
                return CanBusMessage(ResetMessage::deserialize(buffer));
            case RocketStateMessage::MESSAGE_TYPE:
                return CanBusMessage(RocketStateMessage::deserialize(buffer));
            case UnixTimeMessage::MESSAGE_TYPE:
                return CanBusMessage(UnixTimeMessage::deserialize(buffer));
            case VLStatusMessage::MESSAGE_TYPE:
                return CanBusMessage(VLStatusMessage::deserialize(buffer));
            default:
                return std::nullopt;
        }
    }

    struct ReceivedCanBusMessage {
        uint32_t id;
        uint16_t crc;
        CanBusMessage message;
    };

    namespace detail {
        class StateMachine {
        public:
            enum class Type { Empty, MultiFrame };

            StateMachine() noexcept : type(Type::Empty) {}

            bool has_same_id(uint32_t id) const noexcept {
                return type == Type::MultiFrame && multi_frame.id == id;
            }

            std::optional<ReceivedCanBusMessage> process_frame(uint32_t frame_id, const uint8_t* frame_data, size_t frame_len, uint64_t timestamp_us) noexcept {
                if (frame_len == 0) return std::nullopt;

                TailByte tail_byte = TailByte::from_byte(frame_data[frame_len - 1]);

                if (tail_byte.start_of_transfer && tail_byte.end_of_transfer) {
                    if (tail_byte.toggle) return std::nullopt;

                    size_t data_len = frame_len - 1;
                    uint8_t message_type = CanBusExtendedId::message_type_from_raw(frame_id);
                    auto decoded = decode(message_type, frame_data, data_len);
                    if (decoded) {
                        return ReceivedCanBusMessage{frame_id, calculate_crc(frame_data, data_len), *decoded};
                    }
                    return std::nullopt;
                }

                if (type == Type::Empty) {
                    if (!(tail_byte.start_of_transfer && !tail_byte.end_of_transfer && !tail_byte.toggle)) {
                        return std::nullopt;
                    }

                    if (frame_len < 4) return std::nullopt;

                    type = Type::MultiFrame;
                    multi_frame.id = frame_id;
                    multi_frame.first_frame_timestamp_us = timestamp_us;
                    multi_frame.crc = static_cast<uint16_t>(frame_data[0]) | (static_cast<uint16_t>(frame_data[1]) << 8);
                    multi_frame.data_len = frame_len - 3;
                    std::memcpy(multi_frame.data, frame_data + 2, multi_frame.data_len);
                    return std::nullopt;
                } else {
                    if (multi_frame.id != frame_id) {
                        type = Type::Empty;
                        return process_frame(frame_id, frame_data, frame_len, timestamp_us);
                    }

                    bool expected_toggle_bit = ((multi_frame.data_len - 5) / 7) % 2 == 0;
                    if (tail_byte.toggle != expected_toggle_bit) return std::nullopt;
                    if (tail_byte.start_of_transfer) return std::nullopt;

                    size_t new_data_len = frame_len - 1;
                    if (multi_frame.data_len + new_data_len > MAX_CAN_MESSAGE_SIZE) {
                        type = Type::Empty;
                        return std::nullopt;
                    }

                    std::memcpy(multi_frame.data + multi_frame.data_len, frame_data, new_data_len);
                    multi_frame.data_len += new_data_len;

                    if (tail_byte.end_of_transfer) {
                        uint16_t calculated_crc = calculate_crc(multi_frame.data, multi_frame.data_len);
                        if (calculated_crc != multi_frame.crc) {
                            type = Type::Empty;
                            return std::nullopt;
                        }

                        uint8_t message_type = CanBusExtendedId::message_type_from_raw(multi_frame.id);
                        auto decoded = decode(message_type, multi_frame.data, multi_frame.data_len);
                        ReceivedCanBusMessage result_msg;
                        bool success = false;
                        if (decoded) {
                            result_msg = ReceivedCanBusMessage{multi_frame.id, calculated_crc, *decoded};
                            success = true;
                        }
                        type = Type::Empty;
                        if (success) return result_msg;
                    }
                    return std::nullopt;
                }
            }

            uint64_t get_first_frame_timestamp_us() const noexcept {
                return type == Type::MultiFrame ? multi_frame.first_frame_timestamp_us : 0;
            }

            bool is_empty() const noexcept { return type == Type::Empty; }

        private:
            Type type;
            struct {
                uint32_t id;
                uint64_t first_frame_timestamp_us;
                uint16_t crc;
                uint8_t data[MAX_CAN_MESSAGE_SIZE];
                size_t data_len;
            } multi_frame;

            static uint16_t calculate_crc(const uint8_t* data, size_t len) noexcept {
                return can_crc16(data, len);
            }
        };
    } // namespace detail

    class CanBusMultiFrameDecoder {
    public:
        static constexpr size_t Q = 8;

        CanBusMultiFrameDecoder() noexcept {}

        std::optional<ReceivedCanBusMessage> process_frame(uint32_t frame_id, const uint8_t* frame_data, size_t frame_len, uint64_t timestamp_us) noexcept {
            uint8_t message_type = CanBusExtendedId::message_type_from_raw(frame_id);
            if (message_type == LOG_MESSAGE_TYPE) {
                // Log frames are a raw byte stream, not an encoded message, and
                // are handled elsewhere. The decoder simply ignores them.
                return std::nullopt;
            }

            for (size_t i = 0; i < Q; ++i) {
                if (state_machines[i].has_same_id(frame_id)) {
                    return state_machines[i].process_frame(frame_id, frame_data, frame_len, timestamp_us);
                }
            }

            size_t lru_index = 0;
            bool found_empty = false;
            for (size_t i = 0; i < Q; ++i) {
                if (state_machines[i].is_empty()) {
                    lru_index = i;
                    found_empty = true;
                    break;
                }
            }

            if (!found_empty) {
                uint64_t min_ts = state_machines[0].get_first_frame_timestamp_us();
                for (size_t i = 1; i < Q; ++i) {
                    uint64_t ts = state_machines[i].get_first_frame_timestamp_us();
                    if (ts < min_ts) {
                        min_ts = ts;
                        lru_index = i;
                    }
                }
            }

            return state_machines[lru_index].process_frame(frame_id, frame_data, frame_len, timestamp_us);
        }

    private:
        detail::StateMachine state_machines[Q];
    };

} // namespace can_bus
} // namespace firmware_common
