#pragma once

#include <cstdint>
#include <cstring>
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

    struct AckMessage {
        static constexpr uint32_t MESSAGE_TYPE = 66;
        // Message Type ID to be verified against JSON
        static constexpr size_t SIZE_BYTES = 4;

        uint16_t crc;
        uint16_t node_id; // 12 bits

        AckMessage(uint16_t _crc = 0, uint16_t _node_id = 0) noexcept : crc(_crc), node_id(_node_id) {}

        static constexpr uint8_t PRIORITY = 4;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        PowerOutputOverwrite out1;
        PowerOutputOverwrite out2;
        PowerOutputOverwrite out3;
        PowerOutputOverwrite out4;

        AmpOverwriteMessage(PowerOutputOverwrite o1 = PowerOutputOverwrite::NoOverwrite,
                            PowerOutputOverwrite o2 = PowerOutputOverwrite::NoOverwrite,
                            PowerOutputOverwrite o3 = PowerOutputOverwrite::NoOverwrite,
                            PowerOutputOverwrite o4 = PowerOutputOverwrite::NoOverwrite) noexcept
            : out1(o1), out2(o2), out3(o3), out4(o4) {}

        static constexpr uint8_t PRIORITY = 2;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            uint8_t b = 0;
            b |= (static_cast<uint8_t>(out1) & 0x03) << 6;
            b |= (static_cast<uint8_t>(out2) & 0x03) << 4;
            b |= (static_cast<uint8_t>(out3) & 0x03) << 2;
            b |= (static_cast<uint8_t>(out4) & 0x03);
            buffer[0] = b;
        }

        static AmpOverwriteMessage deserialize(const uint8_t* buffer) noexcept {
            AmpOverwriteMessage msg;
            uint8_t b = buffer[0];
            msg.out1 = static_cast<PowerOutputOverwrite>((b >> 6) & 0x03);
            msg.out2 = static_cast<PowerOutputOverwrite>((b >> 4) & 0x03);
            msg.out3 = static_cast<PowerOutputOverwrite>((b >> 2) & 0x03);
            msg.out4 = static_cast<PowerOutputOverwrite>(b & 0x03);
            return msg;
        }
    };

    struct AmpResetOutputMessage {
        static constexpr uint32_t MESSAGE_TYPE = 68;
        static constexpr size_t SIZE_BYTES = 1;
        uint8_t output;

        AmpResetOutputMessage(uint8_t out = 0) noexcept : output(out) {}

        static constexpr uint8_t PRIORITY = 2;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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
        static constexpr size_t SIZE_BYTES = 6;
        
        uint16_t shared_battery_mv;
        AmpOutputStatus out1;
        AmpOutputStatus out2;
        AmpOutputStatus out3;
        AmpOutputStatus out4;

        static constexpr uint8_t PRIORITY = 5;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            std::memset(buffer, 0, SIZE_BYTES);
            write_u16_be(buffer, shared_battery_mv);
            buffer[2] = out1.to_byte();
            buffer[3] = out2.to_byte();
            buffer[4] = out3.to_byte();
            buffer[5] = out4.to_byte();
        }

        static AmpStatusMessage deserialize(const uint8_t* buffer) noexcept {
            AmpStatusMessage msg;
            msg.shared_battery_mv = read_u16_be(buffer);
            msg.out1 = AmpOutputStatus::from_byte(buffer[2]);
            msg.out2 = AmpOutputStatus::from_byte(buffer[3]);
            msg.out3 = AmpOutputStatus::from_byte(buffer[4]);
            msg.out4 = AmpOutputStatus::from_byte(buffer[5]);
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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

    enum class DataType : uint8_t {
        Firmware = 0,
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
            start_of_transfer(false), end_of_transfer(false), data_type(DataType::Firmware), destination_node_id(0) {}

        static constexpr uint8_t PRIORITY = 6;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

    struct OzysMeasurementMessage {
        static constexpr uint32_t MESSAGE_TYPE = 133;
        static constexpr size_t SIZE_BYTES = 16;
        
        uint32_t sg_1_raw;
        uint32_t sg_2_raw;
        uint32_t sg_3_raw;
        uint32_t sg_4_raw;

        static constexpr uint8_t PRIORITY = 5;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

    struct PayloadEPSOutputOverwriteMessage {
        static constexpr uint32_t MESSAGE_TYPE = 65;
        static constexpr size_t SIZE_BYTES = 3;

        PowerOutputOverwrite out_3v3;
        PowerOutputOverwrite out_5v;
        PowerOutputOverwrite out_9v;
        uint16_t node_id; // 12 bits

        static constexpr uint8_t PRIORITY = 2;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            // Byte 0: 3v3(2), 5v(2), 9v(2), node_id_hi(2)
            uint8_t b0 = 0;
            b0 |= (static_cast<uint8_t>(out_3v3) & 0x03) << 6;
            b0 |= (static_cast<uint8_t>(out_5v) & 0x03) << 4;
            b0 |= (static_cast<uint8_t>(out_9v) & 0x03) << 2;
            b0 |= (node_id >> 10) & 0x03;
            buffer[0] = b0;
            
            // Byte 1: node_id_mid(8) -> bits 9..2 of node_id
            buffer[1] = (node_id >> 2) & 0xFF;
            
            // Byte 2: node_id_lo(2) -> bits 1..0 of node_id in high bits
            buffer[2] = (node_id & 0x03) << 6;
        }

        static PayloadEPSOutputOverwriteMessage deserialize(const uint8_t* buffer) noexcept {
            PayloadEPSOutputOverwriteMessage msg;
            uint8_t b0 = buffer[0];
            msg.out_3v3 = static_cast<PowerOutputOverwrite>((b0 >> 6) & 0x03);
            msg.out_5v = static_cast<PowerOutputOverwrite>((b0 >> 4) & 0x03);
            msg.out_9v = static_cast<PowerOutputOverwrite>((b0 >> 2) & 0x03);
            
            uint16_t nid = (b0 & 0x03) << 10;
            nid |= static_cast<uint16_t>(buffer[1]) << 2;
            nid |= (buffer[2] >> 6) & 0x03;
            msg.node_id = nid;
            return msg;
        }
    };

    struct PayloadEPSOutputStatus {
        uint16_t current_ma; // 13 bits
        bool overwrote;      // 1 bit
        PowerOutputStatus status; // 2 bits
        
        void serialize(uint8_t* buffer) const noexcept {
            // Byte 0: current_ma[12..5] (8 bits)
            buffer[0] = (current_ma >> 5) & 0xFF;
            
            // Byte 1: current_ma[4..0] (5 bits), overwrote(1), status(2)
            uint8_t b1 = 0;
            b1 |= (current_ma & 0x1F) << 3;
            if (overwrote) b1 |= 0x04;
            b1 |= (static_cast<uint8_t>(status) & 0x03);
            buffer[1] = b1;
        }

        static PayloadEPSOutputStatus deserialize(const uint8_t* buffer) noexcept {
            PayloadEPSOutputStatus msg;
            msg.current_ma = (static_cast<uint16_t>(buffer[0]) << 5) | ((buffer[1] >> 3) & 0x1F);
            msg.overwrote = (buffer[1] & 0x04) != 0;
            msg.status = static_cast<PowerOutputStatus>(buffer[1] & 0x03);
            return msg;
        }
    };

    struct PayloadEPSStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 34;
        static constexpr size_t SIZE_BYTES = 14;

        uint16_t battery1_mv;
        uint16_t battery1_temperature_raw;
        uint16_t battery2_mv;
        uint16_t battery2_temperature_raw;
        
        PayloadEPSOutputStatus output_3v3;
        PayloadEPSOutputStatus output_5v;
        PayloadEPSOutputStatus output_9v;

        static constexpr uint8_t PRIORITY = 5;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            write_u16_be(buffer, battery1_mv);
            write_u16_be(buffer + 2, battery1_temperature_raw);
            write_u16_be(buffer + 4, battery2_mv);
            write_u16_be(buffer + 6, battery2_temperature_raw);
            output_3v3.serialize(buffer + 8);
            output_5v.serialize(buffer + 10);
            output_9v.serialize(buffer + 12);
        }

        static PayloadEPSStatusMessage deserialize(const uint8_t* buffer) noexcept {
            PayloadEPSStatusMessage msg;
            msg.battery1_mv = read_u16_be(buffer);
            msg.battery1_temperature_raw = read_u16_be(buffer + 2);
            msg.battery2_mv = read_u16_be(buffer + 4);
            msg.battery2_temperature_raw = read_u16_be(buffer + 6);
            msg.output_3v3 = PayloadEPSOutputStatus::deserialize(buffer + 8);
            msg.output_5v = PayloadEPSOutputStatus::deserialize(buffer + 10);
            msg.output_9v = PayloadEPSOutputStatus::deserialize(buffer + 12);
            return msg;
        }
    };

    struct ResetMessage {
        static constexpr uint32_t MESSAGE_TYPE = 0;
        static constexpr size_t SIZE_BYTES = 2;

        uint16_t node_id; // 12 bits
        bool reset_all;
        bool into_bootloader;

        static constexpr uint8_t PRIORITY = 0;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            // Byte 0: node_id[11..4]
            buffer[0] = (node_id >> 4) & 0xFF;
            
            // Byte 1: node_id[3..0], reset_all, into_bootloader, padding(2)
            uint8_t b1 = 0;
            b1 |= (node_id & 0x0F) << 4;
            if (reset_all) b1 |= 0x08;
            if (into_bootloader) b1 |= 0x04;
            buffer[1] = b1;
        }

        static ResetMessage deserialize(const uint8_t* buffer) noexcept {
            ResetMessage msg;
            msg.node_id = (static_cast<uint16_t>(buffer[0]) << 4) | ((buffer[1] >> 4) & 0x0F);
            msg.reset_all = (buffer[1] & 0x08) != 0;
            msg.into_bootloader = (buffer[1] & 0x04) != 0;
            return msg;
        }
    };

    struct RocketStateMessage {
        static constexpr uint32_t MESSAGE_TYPE = 131;
        static constexpr size_t SIZE_BYTES = 20;

        uint32_t velocity_raw[2];
        uint32_t altitude_agl_raw;
        uint64_t timestamp_us;
        bool is_coasting;

        static RocketStateMessage new_msg(uint64_t ts, const float vel[2], float alt, bool coasting) noexcept {
            RocketStateMessage msg;
            msg.timestamp_us = ts;
            for(int i=0; i<2; i++) {
                msg.velocity_raw[i] = bit_cast<uint32_t>(vel[i]);
            }
            msg.altitude_agl_raw = bit_cast<uint32_t>(alt);
            msg.is_coasting = coasting;
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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            write_u32_be(buffer, velocity_raw[0]);
            write_u32_be(buffer + 4, velocity_raw[1]);
            write_u32_be(buffer + 8, altitude_agl_raw);
            write_u56_be(buffer + 12, timestamp_us);
            buffer[19] = is_coasting ? 0x80 : 0x00;
        }

        static RocketStateMessage deserialize(const uint8_t* buffer) noexcept {
            RocketStateMessage msg;
            msg.velocity_raw[0] = read_u32_be(buffer);
            msg.velocity_raw[1] = read_u32_be(buffer + 4);
            msg.altitude_agl_raw = read_u32_be(buffer + 8);
            msg.timestamp_us = read_u56_be(buffer + 12);
            msg.is_coasting = (buffer[19] & 0x80) != 0;
            return msg;
        }
    };

    struct UnixTimeMessage {
        static constexpr uint32_t MESSAGE_TYPE = 7;
        static constexpr size_t SIZE_BYTES = 7;
        uint64_t timestamp_us;

        static constexpr uint8_t PRIORITY = 1;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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
        PoweredAscent = 3,
        Coasting = 4,
        DrogueDeployed = 5,
        MainDeployed = 6,
        Landed = 7
    };

    struct VLStatusMessage {
        static constexpr uint32_t MESSAGE_TYPE = 36;
        static constexpr size_t SIZE_BYTES = 5;

        FlightStage flight_stage;
        uint16_t battery_mv;

        static constexpr uint8_t PRIORITY = 2;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

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

        bool out1_enable;
        bool out2_enable;
        bool out3_enable;
        bool out4_enable;

        AmpControlMessage(bool o1 = false, bool o2 = false, bool o3 = false, bool o4 = false) noexcept
            : out1_enable(o1), out2_enable(o2), out3_enable(o3), out4_enable(o4) {}

        static constexpr uint8_t PRIORITY = 2;

        uint32_t get_id(uint8_t node_type, uint16_t node_id) const noexcept {
            return CanBusExtendedId::create(PRIORITY, MESSAGE_TYPE, node_type, node_id);
        }

        void serialize(uint8_t* buffer) const noexcept {
            uint8_t byte = 0;
            if (out1_enable) byte |= (1 << 7); // MSB0 bit 0 -> 7 in LSB
            if (out2_enable) byte |= (1 << 6);
            if (out3_enable) byte |= (1 << 5);
            if (out4_enable) byte |= (1 << 4);
            // Remaining bits are 0
            buffer[0] = byte;
        }

        static AmpControlMessage deserialize(const uint8_t* buffer) noexcept {
            AmpControlMessage msg;
            uint8_t byte = buffer[0];
            msg.out1_enable = (byte & (1 << 7)) != 0;
            msg.out2_enable = (byte & (1 << 6)) != 0;
            msg.out3_enable = (byte & (1 << 5)) != 0;
            msg.out4_enable = (byte & (1 << 4)) != 0;
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

    struct PreUnixTimeMessage {
        static constexpr uint32_t MESSAGE_TYPE = 8;
        static constexpr size_t SIZE_BYTES = 0;
        
        static PreUnixTimeMessage deserialize(const uint8_t* buffer) noexcept {
            (void)buffer;
            return PreUnixTimeMessage();
        }

        void serialize(uint8_t* buffer) const noexcept {
            (void)buffer;
        }
    };

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
        DataTransferMessage,
        IcarusStatusMessage,
        IMUMeasurementMessage,
        MagMeasurementMessage,
        NodeStatusMessage,
        OzysMeasurementMessage,
        PayloadEPSOutputOverwriteMessage,
        PayloadEPSStatusMessage,
        PreUnixTimeMessage,
        ResetMessage,
        RocketStateMessage,
        UnixTimeMessage,
        VLStatusMessage
    >;

    class CanBusMultiFrameEncoder {
    public:
        static constexpr size_t MAX_CAN_MESSAGE_SIZE = 64;

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
            // CRC-16/IBM-3740: poly=0x1021 init=0xFFFF refin=false refout=false xorout=0x0000
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
    };

    inline std::optional<CanBusMessage> decode(uint8_t message_type, const uint8_t* buffer) noexcept {
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
            case PayloadEPSOutputOverwriteMessage::MESSAGE_TYPE:
                return CanBusMessage(PayloadEPSOutputOverwriteMessage::deserialize(buffer));
            case PayloadEPSStatusMessage::MESSAGE_TYPE:
                return CanBusMessage(PayloadEPSStatusMessage::deserialize(buffer));
            case PreUnixTimeMessage::MESSAGE_TYPE:
                return CanBusMessage(PreUnixTimeMessage::deserialize(buffer));
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
                    uint8_t message_type = (frame_id >> 18) & 0xFF;
                    auto decoded = decode(message_type, frame_data);
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
                    if (multi_frame.data_len + new_data_len > 256) {
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

                        uint8_t message_type = (multi_frame.id >> 18) & 0xFF;
                        auto decoded = decode(message_type, multi_frame.data);
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
                uint8_t data[256];
                size_t data_len;
            } multi_frame;

            static uint16_t calculate_crc(const uint8_t* data, size_t len) noexcept {
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
        };
    } // namespace detail

    class CanBusMultiFrameDecoder {
    public:
        static constexpr size_t Q = 8;

        CanBusMultiFrameDecoder() noexcept {}

        std::optional<ReceivedCanBusMessage> process_frame(uint32_t frame_id, const uint8_t* frame_data, size_t frame_len, uint64_t timestamp_us) noexcept {
            uint8_t message_type = (frame_id >> 18) & 0xFF;
            if (message_type == 255) { // LOG_MESSAGE_TYPE is 255 in Rust
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
