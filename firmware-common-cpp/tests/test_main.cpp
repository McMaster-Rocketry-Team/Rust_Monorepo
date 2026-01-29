#include <gtest/gtest.h>
#include <fstream>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include "firmware_common.hpp"

using json = nlohmann::json;

// Helper to read JSON file
json read_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open file: " + path);
    }
    json data = json::parse(f);
    return data;
}

// Helper to convert vector<int> to vector<uint8_t> from JSON
std::vector<uint8_t> get_bytes(const json& j) {
    std::vector<uint8_t> bytes;
    for (auto& element : j) {
        bytes.push_back(static_cast<uint8_t>(element.get<int>()));
    }
    return bytes;
}

// Helper to resolve path
std::string resolve_path(const std::string& filename) {
    std::vector<std::string> prefixes = {
        "firmware-common-new/can_bus_reference_data/",
        "../firmware-common-new/can_bus_reference_data/",
        "../../firmware-common-new/can_bus_reference_data/",
        "../../../firmware-common-new/can_bus_reference_data/"
    };
    
    for (const auto& prefix : prefixes) {
        std::string path = prefix + filename;
        std::ifstream f(path);
        if (f.good()) return path;
    }
    
    throw std::runtime_error("Could not find file: " + filename);
}

// Helper to verify encoder output
void check_encoder(const firmware_common::can_bus::CanBusMessage& message, const json& item, const std::string& msg_key) {
    if (!item.contains("encoded_data")) return;

    auto expected_encoded = item["encoded_data"];
    firmware_common::can_bus::CanBusMultiFrameEncoder encoder(message);
    
    size_t frame_idx = 0;
    while (encoder.has_next()) {
        auto frame = encoder.next();
        ASSERT_LT(frame_idx, expected_encoded.size()) << "Too many frames from encoder for " << msg_key;
        
        auto expected_frame_bytes = get_bytes(expected_encoded[frame_idx]);
        ASSERT_EQ(frame.len, expected_frame_bytes.size()) << "Frame length mismatch at frame " << frame_idx << " for " << msg_key;
        
        for (size_t i = 0; i < frame.len; ++i) {
            EXPECT_EQ(frame.data[i], expected_frame_bytes[i]) 
                << "Byte mismatch at frame " << frame_idx << ", byte " << i << " for " << msg_key;
        }
        frame_idx++;
    }
    EXPECT_EQ(frame_idx, expected_encoded.size()) << "Too few frames from encoder for " << msg_key;
}

TEST(AirBrakesControlTest, ReferenceData) {
    json data = read_json(resolve_path("airbrakes_control.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["AirBrakesControl"];
        uint16_t expected_extension = message_content["extension_percentage"];
        uint32_t expected_id = item["frame_id"];

        auto msg = firmware_common::can_bus::AirBrakesControlMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.extension_percentage, expected_extension);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AirBrakesControlMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "AirBrakesControl");
    }
}

TEST(AmpControlTest, ReferenceData) {
    json data = read_json(resolve_path("amp_control.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["AmpControl"];
        uint32_t expected_id = item["frame_id"];
        
        bool expected_out1 = message_content["out1_enable"];
        bool expected_out2 = message_content["out2_enable"];
        bool expected_out3 = message_content["out3_enable"];
        bool expected_out4 = message_content["out4_enable"];

        auto msg = firmware_common::can_bus::AmpControlMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.out1_enable, expected_out1);
        EXPECT_EQ(msg.out2_enable, expected_out2);
        EXPECT_EQ(msg.out3_enable, expected_out3);
        EXPECT_EQ(msg.out4_enable, expected_out4);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AmpControlMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "AmpControl");
    }
}

TEST(AckTest, ReferenceData) {
    json data = read_json(resolve_path("ack.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["Ack"];
        uint16_t expected_crc = message_content["crc"];
        uint16_t expected_node_id = message_content["node_id"];
        uint32_t expected_id = item["frame_id"];

        auto msg = firmware_common::can_bus::AckMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.crc, expected_crc);
        EXPECT_EQ(msg.node_id, expected_node_id);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AckMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]) << "Mismatch at byte " << i;
        
        check_encoder(msg, item, "Ack");
    }
}

TEST(AmpOverwriteTest, ReferenceData) {
    json data = read_json(resolve_path("amp_overwrite.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["AmpOverwrite"];
        uint32_t expected_id = item["frame_id"];
        
        auto parse_enum = [](const std::string& s) {
            if (s == "NoOverwrite") return firmware_common::can_bus::PowerOutputOverwrite::NoOverwrite;
            if (s == "ForceEnabled") return firmware_common::can_bus::PowerOutputOverwrite::ForceEnabled;
            if (s == "ForceDisabled") return firmware_common::can_bus::PowerOutputOverwrite::ForceDisabled;
            throw std::runtime_error("Unknown enum value: " + s);
        };

        auto msg = firmware_common::can_bus::AmpOverwriteMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.out1, parse_enum(message_content["out1"]));
        EXPECT_EQ(msg.out2, parse_enum(message_content["out2"]));
        EXPECT_EQ(msg.out3, parse_enum(message_content["out3"]));
        EXPECT_EQ(msg.out4, parse_enum(message_content["out4"]));
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AmpOverwriteMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "AmpOverwrite");
    }
}

TEST(AmpResetOutputTest, ReferenceData) {
    json data = read_json(resolve_path("amp_reset_output.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["AmpResetOutput"];
        uint8_t expected_output = message_content["output"];
        uint32_t expected_id = item["frame_id"];

        auto msg = firmware_common::can_bus::AmpResetOutputMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.output, expected_output);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AmpResetOutputMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "AmpResetOutput");
    }
}

TEST(AmpStatusTest, ReferenceData) {
    json data = read_json(resolve_path("amp_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["AmpStatus"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t expected_battery = message_content["shared_battery_mv"];
        
        auto parse_status_enum = [](const std::string& s) {
            if (s == "Disabled") return firmware_common::can_bus::PowerOutputStatus::Disabled;
            if (s == "PowerGood") return firmware_common::can_bus::PowerOutputStatus::PowerGood;
            if (s == "PowerBad") return firmware_common::can_bus::PowerOutputStatus::PowerBad;
            throw std::runtime_error("Unknown status enum: " + s);
        };

        auto msg = firmware_common::can_bus::AmpStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.shared_battery_mv, expected_battery);
        
        EXPECT_EQ(msg.out1.overwrote, message_content["out1"]["overwrote"]);
        EXPECT_EQ(msg.out1.status, parse_status_enum(message_content["out1"]["status"]));

        EXPECT_EQ(msg.out2.overwrote, message_content["out2"]["overwrote"]);
        EXPECT_EQ(msg.out2.status, parse_status_enum(message_content["out2"]["status"]));

        EXPECT_EQ(msg.out3.overwrote, message_content["out3"]["overwrote"]);
        EXPECT_EQ(msg.out3.status, parse_status_enum(message_content["out3"]["status"]));

        EXPECT_EQ(msg.out4.overwrote, message_content["out4"]["overwrote"]);
        EXPECT_EQ(msg.out4.status, parse_status_enum(message_content["out4"]["status"]));
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::AmpStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "AmpStatus");
    }
}

TEST(BaroMeasurementTest, ReferenceData) {
    json data = read_json(resolve_path("baro_measurement.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["BaroMeasurement"];
        uint32_t expected_id = item["frame_id"];
        
        uint32_t expected_pressure_raw = message_content["pressure_raw"];
        uint16_t expected_temp_raw = message_content["temperature_raw"];
        uint64_t expected_timestamp = message_content["timestamp_us"];

        auto msg = firmware_common::can_bus::BaroMeasurementMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.pressure_raw, expected_pressure_raw);
        EXPECT_EQ(msg.temperature_raw, expected_temp_raw);
        EXPECT_EQ(msg.timestamp_us, expected_timestamp);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::BaroMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "BaroMeasurement");
    }
}

TEST(BrightnessMeasurementTest, ReferenceData) {
    json data = read_json(resolve_path("brightness_measurement.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["BrightnessMeasurement"];
        uint32_t expected_id = item["frame_id"];
        
        uint32_t expected_lux_raw = message_content["brightness_lux_raw"];
        uint64_t expected_timestamp = message_content["timestamp_us"];

        auto msg = firmware_common::can_bus::BrightnessMeasurementMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.brightness_lux_raw, expected_lux_raw);
        EXPECT_EQ(msg.timestamp_us, expected_timestamp);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::BrightnessMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "BrightnessMeasurement");
    }
}

TEST(DataTransferTest, ReferenceData) {
    json data = read_json(resolve_path("data_transfer.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["DataTransfer"];
        uint32_t expected_id = item["frame_id"];
        
        auto expected_data_vec = get_bytes(message_content["data"]);
        uint8_t expected_data_len = message_content["data_len"];
        uint8_t expected_seq = message_content["sequence_number"];
        bool expected_start = message_content["start_of_transfer"];
        bool expected_end = message_content["end_of_transfer"];
        uint16_t expected_node_id = message_content["destination_node_id"];
        
        std::string dt_str = message_content["data_type"];
        firmware_common::can_bus::DataType expected_type = 
            (dt_str == "Firmware") ? firmware_common::can_bus::DataType::Firmware : firmware_common::can_bus::DataType::Data;

        auto msg = firmware_common::can_bus::DataTransferMessage::deserialize(serialized_data.data());
        
        for(size_t i=0; i<32; i++) EXPECT_EQ(msg.data[i], expected_data_vec[i]);
        EXPECT_EQ(msg.data_len, expected_data_len);
        EXPECT_EQ(msg.sequence_number, expected_seq);
        EXPECT_EQ(msg.start_of_transfer, expected_start);
        EXPECT_EQ(msg.end_of_transfer, expected_end);
        EXPECT_EQ(msg.data_type, expected_type);
        EXPECT_EQ(msg.destination_node_id, expected_node_id);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::DataTransferMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]) << "Mismatch at byte " << i;
        
        check_encoder(msg, item, "DataTransfer");
    }
}

TEST(IcarusStatusTest, ReferenceData) {
    json data = read_json(resolve_path("icarus_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["IcarusStatus"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t expected_ext = message_content["actual_extension_percentage"];
        uint16_t expected_temp = message_content["servo_temperature_raw"];
        uint16_t expected_curr = message_content["servo_current_raw"];

        auto msg = firmware_common::can_bus::IcarusStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.actual_extension_percentage, expected_ext);
        EXPECT_EQ(msg.servo_temperature_raw, expected_temp);
        EXPECT_EQ(msg.servo_current_raw, expected_curr);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::IcarusStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "IcarusStatus");
    }
}

TEST(ImuMeasurementTest, ReferenceData) {
    json data = read_json(resolve_path("imu_measurement.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["IMUMeasurement"];
        uint32_t expected_id = item["frame_id"];
        
        std::vector<uint32_t> expected_acc;
        for(auto& x : message_content["acc_raw"]) expected_acc.push_back(x.get<uint32_t>());
        
        std::vector<uint32_t> expected_gyro;
        for(auto& x : message_content["gyro_raw"]) expected_gyro.push_back(x.get<uint32_t>());
        
        uint64_t expected_timestamp = message_content["timestamp_us"];

        auto msg = firmware_common::can_bus::IMUMeasurementMessage::deserialize(serialized_data.data());
        for(int i=0; i<3; i++) EXPECT_EQ(msg.acc_raw[i], expected_acc[i]);
        for(int i=0; i<3; i++) EXPECT_EQ(msg.gyro_raw[i], expected_gyro[i]);
        EXPECT_EQ(msg.timestamp_us, expected_timestamp);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::IMUMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "IMUMeasurement");
    }
}

TEST(MagMeasurementTest, ReferenceData) {
    json data = read_json(resolve_path("mag_measurement.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["MagMeasurement"];
        uint32_t expected_id = item["frame_id"];
        
        std::vector<uint32_t> expected_mag;
        for(auto& x : message_content["mag_raw"]) expected_mag.push_back(x.get<uint32_t>());
        
        uint64_t expected_timestamp = message_content["timestamp_us"];

        auto msg = firmware_common::can_bus::MagMeasurementMessage::deserialize(serialized_data.data());
        for(int i=0; i<3; i++) EXPECT_EQ(msg.mag_raw[i], expected_mag[i]);
        EXPECT_EQ(msg.timestamp_us, expected_timestamp);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::MagMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "MagMeasurement");
    }
}

TEST(NodeStatusTest, ReferenceData) {
    json data = read_json(resolve_path("node_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["NodeStatus"];
        uint32_t expected_id = item["frame_id"];
        
        uint32_t expected_uptime = message_content["uptime_s"];
        uint16_t expected_custom = message_content["custom_status_raw"];
        
        auto parse_health = [](const std::string& s) {
            if (s == "Healthy") return firmware_common::can_bus::NodeHealth::Healthy;
            if (s == "Warning") return firmware_common::can_bus::NodeHealth::Warning;
            if (s == "Error") return firmware_common::can_bus::NodeHealth::Error;
            if (s == "Critical") return firmware_common::can_bus::NodeHealth::Critical;
            throw std::runtime_error("Unknown health: " + s);
        };
        auto parse_mode = [](const std::string& s) {
            if (s == "Operational") return firmware_common::can_bus::NodeMode::Operational;
            if (s == "Initialization") return firmware_common::can_bus::NodeMode::Initialization;
            if (s == "Maintenance") return firmware_common::can_bus::NodeMode::Maintenance;
            if (s == "Offline") return firmware_common::can_bus::NodeMode::Offline;
            throw std::runtime_error("Unknown mode: " + s);
        };

        auto msg = firmware_common::can_bus::NodeStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.uptime_s, expected_uptime);
        EXPECT_EQ(msg.custom_status_raw, expected_custom);
        EXPECT_EQ(msg.health, parse_health(message_content["health"]));
        EXPECT_EQ(msg.mode, parse_mode(message_content["mode"]));
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::NodeStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "NodeStatus");
    }
}

TEST(OzysMeasurementTest, ReferenceData) {
    json data = read_json(resolve_path("ozys_measurement.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["OzysMeasurement"];
        uint32_t expected_id = item["frame_id"];
        
        uint32_t sg1 = message_content["sg_1_raw"];
        uint32_t sg2 = message_content["sg_2_raw"];
        uint32_t sg3 = message_content["sg_3_raw"];
        uint32_t sg4 = message_content["sg_4_raw"];

        auto msg = firmware_common::can_bus::OzysMeasurementMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.sg_1_raw, sg1);
        EXPECT_EQ(msg.sg_2_raw, sg2);
        EXPECT_EQ(msg.sg_3_raw, sg3);
        EXPECT_EQ(msg.sg_4_raw, sg4);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::OzysMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "OzysMeasurement");
    }
}

TEST(PayloadEPSOutputOverwriteTest, ReferenceData) {
    json data = read_json(resolve_path("payload_eps_output_overwrite.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["PayloadEPSOutputOverwrite"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t expected_node_id = message_content["node_id"];
        
        auto parse_enum = [](const std::string& s) {
            if (s == "NoOverwrite") return firmware_common::can_bus::PowerOutputOverwrite::NoOverwrite;
            if (s == "ForceEnabled") return firmware_common::can_bus::PowerOutputOverwrite::ForceEnabled;
            if (s == "ForceDisabled") return firmware_common::can_bus::PowerOutputOverwrite::ForceDisabled;
            throw std::runtime_error("Unknown enum value: " + s);
        };

        auto msg = firmware_common::can_bus::PayloadEPSOutputOverwriteMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.node_id, expected_node_id);
        EXPECT_EQ(msg.out_3v3, parse_enum(message_content["out_3v3"]));
        EXPECT_EQ(msg.out_5v, parse_enum(message_content["out_5v"]));
        EXPECT_EQ(msg.out_9v, parse_enum(message_content["out_9v"]));
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::PayloadEPSOutputOverwriteMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "PayloadEPSOutputOverwrite");
    }
}

TEST(PayloadEPSStatusTest, ReferenceData) {
    json data = read_json(resolve_path("payload_eps_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["PayloadEPSStatus"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t b1_mv = message_content["battery1_mv"];
        uint16_t b1_t = message_content["battery1_temperature_raw"];
        uint16_t b2_mv = message_content["battery2_mv"];
        uint16_t b2_t = message_content["battery2_temperature_raw"];
        
        auto check_output = [](const firmware_common::can_bus::PayloadEPSOutputStatus& status, const json& j) {
            uint16_t expected_curr = j["current_ma"];
            bool expected_overwrote = j["overwrote"];
            std::string s = j["status"];
            firmware_common::can_bus::PowerOutputStatus expected_status;
            if (s == "Disabled") expected_status = firmware_common::can_bus::PowerOutputStatus::Disabled;
            else if (s == "PowerGood") expected_status = firmware_common::can_bus::PowerOutputStatus::PowerGood;
            else if (s == "PowerBad") expected_status = firmware_common::can_bus::PowerOutputStatus::PowerBad;
            else throw std::runtime_error("Unknown status: " + s);
            
            EXPECT_EQ(status.current_ma, expected_curr);
            EXPECT_EQ(status.overwrote, expected_overwrote);
            EXPECT_EQ(status.status, expected_status);
        };

        auto msg = firmware_common::can_bus::PayloadEPSStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.battery1_mv, b1_mv);
        EXPECT_EQ(msg.battery1_temperature_raw, b1_t);
        EXPECT_EQ(msg.battery2_mv, b2_mv);
        EXPECT_EQ(msg.battery2_temperature_raw, b2_t);
        
        check_output(msg.output_3v3, message_content["output_3v3"]);
        check_output(msg.output_5v, message_content["output_5v"]);
        check_output(msg.output_9v, message_content["output_9v"]);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::PayloadEPSStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "PayloadEPSStatus");
    }
}

TEST(ResetTest, ReferenceData) {
    json data = read_json(resolve_path("reset.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["Reset"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t expected_node_id = message_content["node_id"];
        bool expected_reset = message_content["reset_all"];
        bool expected_boot = message_content["into_bootloader"];

        auto msg = firmware_common::can_bus::ResetMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.node_id, expected_node_id);
        EXPECT_EQ(msg.reset_all, expected_reset);
        EXPECT_EQ(msg.into_bootloader, expected_boot);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::ResetMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "Reset");
    }
}

TEST(RocketStateTest, ReferenceData) {
    json data = read_json(resolve_path("rocket_state.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["RocketState"];
        uint32_t expected_id = item["frame_id"];
        
        uint32_t alt_raw = message_content["altitude_agl_raw"];
        uint64_t ts = message_content["timestamp_us"];
        bool coasting = message_content["is_coasting"];
        std::vector<uint32_t> vel_raw;
        for(auto& x : message_content["velocity_raw"]) vel_raw.push_back(x.get<uint32_t>());

        auto msg = firmware_common::can_bus::RocketStateMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.altitude_agl_raw, alt_raw);
        EXPECT_EQ(msg.timestamp_us, ts);
        EXPECT_EQ(msg.is_coasting, coasting);
        EXPECT_EQ(msg.velocity_raw[0], vel_raw[0]);
        EXPECT_EQ(msg.velocity_raw[1], vel_raw[1]);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::RocketStateMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "RocketState");
    }
}

TEST(UnixTimeTest, ReferenceData) {
    json data = read_json(resolve_path("unix_time.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["UnixTime"];
        uint32_t expected_id = item["frame_id"];
        
        uint64_t ts = message_content["timestamp_us"];

        auto msg = firmware_common::can_bus::UnixTimeMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.timestamp_us, ts);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::UnixTimeMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "UnixTime");
    }
}

TEST(VLStatusTest, ReferenceData) {
    json data = read_json(resolve_path("vl_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["VLStatus"];
        uint32_t expected_id = item["frame_id"];
        
        uint16_t bat_mv = message_content["battery_mv"];
        std::string stage_str = message_content["flight_stage"];
        
        firmware_common::can_bus::FlightStage expected_stage;
        if (stage_str == "LowPower") expected_stage = firmware_common::can_bus::FlightStage::LowPower;
        else if (stage_str == "SelfTest") expected_stage = firmware_common::can_bus::FlightStage::SelfTest;
        else if (stage_str == "Armed") expected_stage = firmware_common::can_bus::FlightStage::Armed;
        else if (stage_str == "PoweredAscent") expected_stage = firmware_common::can_bus::FlightStage::PoweredAscent;
        else if (stage_str == "Coasting") expected_stage = firmware_common::can_bus::FlightStage::Coasting;
        else if (stage_str == "DrogueDeployed") expected_stage = firmware_common::can_bus::FlightStage::DrogueDeployed;
        else if (stage_str == "MainDeployed") expected_stage = firmware_common::can_bus::FlightStage::MainDeployed;
        else if (stage_str == "Landed") expected_stage = firmware_common::can_bus::FlightStage::Landed;
        else throw std::runtime_error("Unknown flight stage: " + stage_str);

        auto msg = firmware_common::can_bus::VLStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.battery_mv, bat_mv);
        EXPECT_EQ(msg.flight_stage, expected_stage);
        EXPECT_EQ(msg.get_id(10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::VLStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "VLStatus");
    }
}

TEST(CanBusMultiFrameDecoderTest, SingleFrame) {
    firmware_common::can_bus::NodeStatusMessage msg(10, firmware_common::can_bus::NodeHealth::Healthy, firmware_common::can_bus::NodeMode::Maintenance, 0);
    uint32_t id = msg.get_id(10, 20);

    firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
    auto frame_data = encoder.next();
    
    firmware_common::can_bus::CanBusMultiFrameDecoder decoder;
    auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 1000);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->id, id);
    EXPECT_TRUE(std::holds_alternative<firmware_common::can_bus::NodeStatusMessage>(decoded->message));
    auto decoded_msg = std::get<firmware_common::can_bus::NodeStatusMessage>(decoded->message);
    EXPECT_EQ(decoded_msg.uptime_s, 10);
}

TEST(CanBusMultiFrameDecoderTest, MultiFrame) {
    // PayloadEPSStatusMessage is 14 bytes, should be multi-frame
    firmware_common::can_bus::PayloadEPSStatusMessage msg;
    msg.battery1_mv = 7400;
    uint32_t id = msg.get_id(10, 20);

    firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
    firmware_common::can_bus::CanBusMultiFrameDecoder decoder;
    std::optional<firmware_common::can_bus::ReceivedCanBusMessage> decoded;

    while (encoder.has_next()) {
        auto frame_data = encoder.next();
        decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 1000);
    }

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->id, id);
    EXPECT_TRUE(std::holds_alternative<firmware_common::can_bus::PayloadEPSStatusMessage>(decoded->message));
    auto decoded_msg = std::get<firmware_common::can_bus::PayloadEPSStatusMessage>(decoded->message);
    EXPECT_EQ(decoded_msg.battery1_mv, 7400);
}

TEST(CanBusMultiFrameDecoderTest, LRUDiscard) {
    firmware_common::can_bus::CanBusMultiFrameDecoder decoder;
    
    // Fill up all 8 state machines with first frames of different IDs
    for (int i = 0; i < 8; ++i) {
        firmware_common::can_bus::PayloadEPSStatusMessage msg; // 14 bytes
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(1, 34, 1, i);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, static_cast<uint64_t>(1000 + i));
        EXPECT_FALSE(decoded.has_value());
    }

    // Now send a 9th ID, it should discard the one with timestamp 1000 (i=0)
    {
        firmware_common::can_bus::PayloadEPSStatusMessage msg;
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(1, 34, 1, 100);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 2000);
        EXPECT_FALSE(decoded.has_value());
    }

    // If we now send the second frame for ID 0, it should fail/restart because it was discarded
    {
        firmware_common::can_bus::PayloadEPSStatusMessage msg;
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        encoder.next(); // skip first
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(1, 34, 1, 0);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 3000);
        EXPECT_FALSE(decoded.has_value());
    }
}

