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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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

        auto msg = firmware_common::can_bus::AmpControlMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.out1_enable, expected_out1);
        EXPECT_EQ(msg.out2_enable, expected_out2);
        EXPECT_EQ(msg.out3_enable, expected_out3);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::BrightnessMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "BrightnessMeasurement");
    }
}

TEST(CustomPayloadStatusTest, ReferenceData) {
    json data = read_json(resolve_path("custom_payload_status.json"));

    for (const auto& item : data) {
        auto serialized_data = get_bytes(item["serialized_data"]);
        auto message_content = item["message"]["CustomPayloadStatus"];
        uint32_t expected_id = item["frame_id"];

        uint16_t expected_batt = message_content["epm_batt_mv"];
        uint16_t expected_sys_3v3 = message_content["epm_sys_3v3_ma"];
        uint16_t expected_sys_5v = message_content["epm_sys_5v_ma"];
        uint16_t expected_per_3v3 = message_content["epm_per_3v3_ma"];
        uint16_t expected_per_5v = message_content["epm_per_5v_ma"];
        uint16_t expected_per_9v = message_content["epm_per_9v_ma"];
        uint16_t expected_per_12v = message_content["epm_per_12v_ma"];
        uint16_t expected_act_1 = message_content["sem_actuator_1_steps"];
        uint16_t expected_act_2 = message_content["sem_actuator_2_steps"];
        uint16_t expected_act_3 = message_content["sem_actuator_3_steps"];

        auto msg = firmware_common::can_bus::CustomPayloadStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.epm_batt_mv, expected_batt);
        EXPECT_EQ(msg.epm_sys_3v3_ma, expected_sys_3v3);
        EXPECT_EQ(msg.epm_sys_5v_ma, expected_sys_5v);
        EXPECT_EQ(msg.epm_per_3v3_ma, expected_per_3v3);
        EXPECT_EQ(msg.epm_per_5v_ma, expected_per_5v);
        EXPECT_EQ(msg.epm_per_9v_ma, expected_per_9v);
        EXPECT_EQ(msg.epm_per_12v_ma, expected_per_12v);
        EXPECT_EQ(msg.sem_actuator_1_steps, expected_act_1);
        EXPECT_EQ(msg.sem_actuator_2_steps, expected_act_2);
        EXPECT_EQ(msg.sem_actuator_3_steps, expected_act_3);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::CustomPayloadStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);

        check_encoder(msg, item, "CustomPayloadStatus");
    }
}

TEST(CustomPayloadStatusTest, ReadingUnavailable) {
    using firmware_common::can_bus::CustomPayloadStatusMessage;

    auto msg = CustomPayloadStatusMessage::new_unavailable();
    EXPECT_EQ(msg.epm_batt_mv, CustomPayloadStatusMessage::PAYLOAD_READING_UNAVAILABLE);
    EXPECT_EQ(msg.epm_per_12v_ma, CustomPayloadStatusMessage::PAYLOAD_READING_UNAVAILABLE);
    EXPECT_EQ(msg.sem_actuator_3_steps, CustomPayloadStatusMessage::PAYLOAD_READING_UNAVAILABLE);
    EXPECT_FALSE(CustomPayloadStatusMessage::reading(msg.epm_batt_mv).has_value());

    EXPECT_EQ(CustomPayloadStatusMessage::reading(0).value(), 0);
    EXPECT_EQ(CustomPayloadStatusMessage::reading(12600).value(), 12600);
}

// Rail index order must match Rust's rail_ma() / actuator_steps(); the SD slow
// record stores the arrays in exactly this order.
TEST(CustomPayloadStatusTest, RailAndActuatorOrder) {
    using firmware_common::can_bus::CustomPayloadStatusMessage;

    CustomPayloadStatusMessage msg{};
    msg.epm_sys_3v3_ma = 10;
    msg.epm_sys_5v_ma = 11;
    msg.epm_per_3v3_ma = 12;
    msg.epm_per_5v_ma = 13;
    msg.epm_per_9v_ma = 14;
    msg.epm_per_12v_ma = 15;
    msg.sem_actuator_1_steps = 100;
    msg.sem_actuator_2_steps = 200;
    msg.sem_actuator_3_steps = 300;

    auto rails = msg.rail_ma();
    for (uint16_t i = 0; i < 6; ++i) EXPECT_EQ(rails[i].value(), 10 + i);

    auto steps = msg.actuator_steps();
    EXPECT_EQ(steps[0].value(), 100);
    EXPECT_EQ(steps[1].value(), 200);
    EXPECT_EQ(steps[2].value(), 300);
}

// A reading that is unavailable has to stay unavailable all the way to the
// caller, and a genuine 0 has to survive as a 0 — a switched-off rail and an
// actuator at its home position both read 0 in normal operation.
TEST(CustomPayloadStatusTest, UnavailableReadingsAreNulloptAndZerosAreNot) {
    using firmware_common::can_bus::CustomPayloadStatusMessage;
    constexpr uint16_t UNAVAILABLE = CustomPayloadStatusMessage::PAYLOAD_READING_UNAVAILABLE;

    auto all_unavailable = CustomPayloadStatusMessage::new_unavailable();
    EXPECT_FALSE(all_unavailable.epm_batt_mv_reading().has_value());
    for (const auto& rail : all_unavailable.rail_ma()) EXPECT_FALSE(rail.has_value());
    for (const auto& step : all_unavailable.actuator_steps()) EXPECT_FALSE(step.has_value());

    // One dead rail and one dead actuator channel, everything else a real 0.
    CustomPayloadStatusMessage msg{};
    msg.epm_sys_5v_ma = UNAVAILABLE;
    msg.sem_actuator_2_steps = UNAVAILABLE;

    EXPECT_EQ(msg.epm_batt_mv_reading().value(), 0);

    auto rails = msg.rail_ma();
    EXPECT_EQ(rails[0].value(), 0);
    EXPECT_FALSE(rails[1].has_value());
    for (size_t i = 2; i < rails.size(); ++i) EXPECT_EQ(rails[i].value(), 0);

    auto steps = msg.actuator_steps();
    EXPECT_EQ(steps[0].value(), 0);
    EXPECT_FALSE(steps[1].has_value());
    EXPECT_EQ(steps[2].value(), 0);

    // The individual accessors agree with the arrays, and the sentinel still
    // goes out on the wire and comes back as std::nullopt.
    EXPECT_FALSE(msg.epm_sys_5v_ma_reading().has_value());
    EXPECT_FALSE(msg.sem_actuator_2_steps_reading().has_value());
    EXPECT_EQ(msg.epm_per_12v_ma_reading().value(), 0);
    EXPECT_EQ(msg.sem_actuator_3_steps_reading().value(), 0);

    uint8_t buffer[CustomPayloadStatusMessage::SIZE_BYTES];
    msg.serialize(buffer);
    auto round_tripped = CustomPayloadStatusMessage::deserialize(buffer);
    EXPECT_EQ(round_tripped.epm_sys_5v_ma, UNAVAILABLE);
    EXPECT_FALSE(round_tripped.epm_sys_5v_ma_reading().has_value());
    EXPECT_EQ(round_tripped.epm_sys_3v3_ma_reading().value(), 0);
}

// Matches Rust's data(): the bytes past data_len are padding.
TEST(DataTransferTest, DataSizeClampsToCapacity) {
    using firmware_common::can_bus::DataTransferMessage;

    DataTransferMessage msg;
    EXPECT_EQ(msg.data_size(), 0u);
    msg.data_len = 5;
    EXPECT_EQ(msg.data_size(), 5u);
    msg.data_len = 200;
    EXPECT_EQ(msg.data_size(), DataTransferMessage::DATA_CAPACITY);
}

// Mirrors payload_sdrm_custom_status.rs: the SDRM's own layout, epm_alive is
// bit 0, bits 8..10 are spare.
TEST(PayloadSDRMCustomStatusTest, PackedBitLayout) {
    using firmware_common::can_bus::PayloadSDRMCustomStatus;

    PayloadSDRMCustomStatus status;
    EXPECT_EQ(status.to_raw(), 0);

    status.epm_alive = true;
    EXPECT_EQ(status.to_raw(), 0b00000001);

    status = PayloadSDRMCustomStatus{};
    status.sem_sd_logging = true;
    EXPECT_EQ(status.to_raw(), 0b10000000);

    status = PayloadSDRMCustomStatus{};
    status.epm_alive = true;
    status.sem_alive = true;
    status.epm_rails_on = true;
    EXPECT_EQ(status.to_raw(), 0b00000111);

    auto round_tripped = PayloadSDRMCustomStatus::from_raw(status.to_raw());
    EXPECT_EQ(round_tripped.to_raw(), status.to_raw());
    EXPECT_TRUE(round_tripped.epm_alive);
    EXPECT_TRUE(round_tripped.epm_rails_on);
    EXPECT_FALSE(round_tripped.exp1_active);
}

// Reference frame shared with the payload team: rails up, all experiments
// active, both SD logs healthy, uptime 120s.
TEST(PayloadSDRMCustomStatusTest, ReferenceNodeStatusFrame) {
    using namespace firmware_common::can_bus;

    PayloadSDRMCustomStatus status;
    status.epm_alive = true;
    status.sem_alive = true;
    status.epm_rails_on = true;
    status.sdrm_sd_logging = true;
    status.sem_sd_logging = true;
    status.exp1_active = true;
    status.exp2_active = true;
    status.exp3_active = true;
    EXPECT_EQ(status.to_raw(), 0xFF);

    NodeStatusMessage msg(120, NodeHealth::Healthy, NodeMode::Operational, status.to_raw());

    uint8_t buffer[NodeStatusMessage::SIZE_BYTES];
    msg.serialize(buffer);
    const uint8_t expected[] = {0x00, 0x00, 0x78, 0x01, 0xFE};
    for (size_t i = 0; i < sizeof(expected); ++i) EXPECT_EQ(buffer[i], expected[i]);
}

TEST(PayloadSDRMCustomStatusTest, ClearFlags) {
    using firmware_common::can_bus::PayloadSDRMCustomStatus;

    PayloadSDRMCustomStatus all_set;
    all_set.epm_alive = true;
    all_set.sem_alive = true;
    all_set.epm_rails_on = true;
    all_set.sdrm_sd_logging = true;
    all_set.sem_sd_logging = true;
    all_set.exp1_active = true;
    all_set.exp2_active = true;
    all_set.exp3_active = true;

    // LowPower safe-reset: experiment flags cleared, everything else kept
    auto status = all_set;
    status.clear_experiment_flags();
    EXPECT_EQ(status.to_raw(), 0b11000111);

    // Landed shutdown: rails and logging cleared too, liveness kept
    status = all_set;
    status.clear_powered_flags();
    EXPECT_EQ(status.to_raw(), 0b00000011);
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

        auto msg = firmware_common::can_bus::DataTransferMessage::deserialize(serialized_data.data());
        
        for(size_t i=0; i<32; i++) EXPECT_EQ(msg.data[i], expected_data_vec[i]);
        EXPECT_EQ(msg.data_len, expected_data_len);
        EXPECT_EQ(msg.sequence_number, expected_seq);
        EXPECT_EQ(msg.start_of_transfer, expected_start);
        EXPECT_EQ(msg.end_of_transfer, expected_end);
        EXPECT_EQ(msg.destination_node_id, expected_node_id);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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

        auto msg = firmware_common::can_bus::IcarusStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.actual_extension_percentage, expected_ext);
        EXPECT_EQ(msg.servo_temperature_raw, expected_temp);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::OzysMeasurementMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "OzysMeasurement");
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

        auto msg = firmware_common::can_bus::ResetMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.node_id, expected_node_id);
        EXPECT_EQ(msg.reset_all, expected_reset);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::ResetMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "Reset");
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
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

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
        else if (stage_str == "Ascent") expected_stage = firmware_common::can_bus::FlightStage::Ascent;
        else if (stage_str == "DrogueChute") expected_stage = firmware_common::can_bus::FlightStage::DrogueChute;
        else if (stage_str == "MainChute") expected_stage = firmware_common::can_bus::FlightStage::MainChute;
        else if (stage_str == "Landed") expected_stage = firmware_common::can_bus::FlightStage::Landed;
        else if (stage_str == "FailedToReachMinApogee") expected_stage = firmware_common::can_bus::FlightStage::FailedToReachMinApogee;
        else throw std::runtime_error("Unknown flight stage: " + stage_str);

        auto msg = firmware_common::can_bus::VLStatusMessage::deserialize(serialized_data.data());
        EXPECT_EQ(msg.battery_mv, bat_mv);
        EXPECT_EQ(msg.flight_stage, expected_stage);
        EXPECT_EQ(firmware_common::can_bus::get_frame_id(msg, 10, 20), expected_id);

        uint8_t buffer[firmware_common::can_bus::VLStatusMessage::SIZE_BYTES];
        msg.serialize(buffer);
        for (size_t i = 0; i < serialized_data.size(); ++i) EXPECT_EQ(buffer[i], serialized_data[i]);
        
        check_encoder(msg, item, "VLStatus");
    }
}

TEST(CanBusMultiFrameDecoderTest, SingleFrame) {
    firmware_common::can_bus::NodeStatusMessage msg(10, firmware_common::can_bus::NodeHealth::Healthy, firmware_common::can_bus::NodeMode::Maintenance, 0);
    uint32_t id = firmware_common::can_bus::get_frame_id(msg, 10, 20);

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
    // CustomPayloadStatusMessage is 20 bytes, should be multi-frame
    auto msg = firmware_common::can_bus::CustomPayloadStatusMessage::new_unavailable();
    msg.epm_batt_mv = 7400;
    uint32_t id = firmware_common::can_bus::get_frame_id(msg, 10, 20);

    firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
    firmware_common::can_bus::CanBusMultiFrameDecoder decoder;
    std::optional<firmware_common::can_bus::ReceivedCanBusMessage> decoded;

    while (encoder.has_next()) {
        auto frame_data = encoder.next();
        decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 1000);
    }

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->id, id);
    EXPECT_TRUE(std::holds_alternative<firmware_common::can_bus::CustomPayloadStatusMessage>(decoded->message));
    auto decoded_msg = std::get<firmware_common::can_bus::CustomPayloadStatusMessage>(decoded->message);
    EXPECT_EQ(decoded_msg.epm_batt_mv, 7400);
}

// Mirrors Rust's PackedStructSlice: a buffer that is not exactly the serialized
// length of the message type decodes to nullopt rather than reading past the end.
TEST(DecodeTest, RejectsWrongLength) {
    using namespace firmware_common::can_bus;

    uint8_t buffer[NodeStatusMessage::SIZE_BYTES] = {0};
    NodeStatusMessage(120, NodeHealth::Healthy, NodeMode::Operational, 0).serialize(buffer);

    EXPECT_TRUE(decode(NodeStatusMessage::MESSAGE_TYPE, buffer, NodeStatusMessage::SIZE_BYTES).has_value());
    EXPECT_FALSE(decode(NodeStatusMessage::MESSAGE_TYPE, buffer, NodeStatusMessage::SIZE_BYTES - 1).has_value());
    EXPECT_FALSE(decode(NodeStatusMessage::MESSAGE_TYPE, buffer, NodeStatusMessage::SIZE_BYTES + 1).has_value());

    // Unknown message type
    EXPECT_FALSE(decode(0xFF, buffer, sizeof(buffer)).has_value());
    EXPECT_FALSE(serialized_len(0xFF).has_value());
}

// Log frames are a raw byte stream and must be ignored by the decoder.
TEST(CanBusMultiFrameDecoderTest, IgnoresLogFrames) {
    using namespace firmware_common::can_bus;

    CanBusMultiFrameDecoder decoder;
    uint32_t id = CanBusExtendedId::create(7, LOG_MESSAGE_TYPE, 10, 20);
    uint8_t frame_data[8] = {'h', 'e', 'l', 'l', 'o', '!', '!', '!'};

    EXPECT_FALSE(decoder.process_frame(id, frame_data, sizeof(frame_data), 1000).has_value());
}

TEST(CanBusExtendedIdTest, FromRawRoundTrip) {
    using namespace firmware_common::can_bus;

    uint32_t raw = CanBusExtendedId::create(5, BaroMeasurementMessage::MESSAGE_TYPE, 10, 20);
    auto id = CanBusExtendedId::from_raw(raw);

    EXPECT_EQ(id.priority, 5);
    EXPECT_EQ(id.message_type, BaroMeasurementMessage::MESSAGE_TYPE);
    EXPECT_EQ(id.node_type, 10);
    EXPECT_EQ(id.node_id, 20);
    EXPECT_EQ(CanBusExtendedId::message_type_from_raw(raw), BaroMeasurementMessage::MESSAGE_TYPE);
}

// An absent strain gauge channel is carried as NaN, same as Rust's Option<f32>.
TEST(CanBusFilterMaskTest, AcceptsListedAndAlwaysOnTypes) {
    using namespace firmware_common::can_bus;

    uint32_t mask = create_can_bus_message_type_filter_mask({
        static_cast<uint8_t>(BaroMeasurementMessage::MESSAGE_TYPE),
        static_cast<uint8_t>(DataTransferMessage::MESSAGE_TYPE),
    });

    // Listed types pass regardless of priority, node type and node id.
    EXPECT_EQ(CanBusExtendedId::create(5, BaroMeasurementMessage::MESSAGE_TYPE, 10, 20) & mask, 0u);
    EXPECT_EQ(CanBusExtendedId::create(1, DataTransferMessage::MESSAGE_TYPE, 20, 30) & mask, 0u);

    // Reset and unix time are always accepted, even though they were not listed.
    EXPECT_EQ(CanBusExtendedId::create(1, ResetMessage::MESSAGE_TYPE, 20, 30) & mask, 0u);
    EXPECT_EQ(CanBusExtendedId::create(1, UnixTimeMessage::MESSAGE_TYPE, 20, 30) & mask, 0u);

    // These two happen to be rejected by this particular mask.
    EXPECT_NE(CanBusExtendedId::create(1, AckMessage::MESSAGE_TYPE, 20, 30) & mask, 0u);
    EXPECT_NE(CanBusExtendedId::create(1, AmpStatusMessage::MESSAGE_TYPE, 20, 30) & mask, 0u);
}

TEST(CanBusFilterMaskTest, PointerAndInitializerListAgree) {
    using namespace firmware_common::can_bus;

    const uint8_t types[] = {
        static_cast<uint8_t>(BaroMeasurementMessage::MESSAGE_TYPE),
        static_cast<uint8_t>(DataTransferMessage::MESSAGE_TYPE),
    };

    EXPECT_EQ(create_can_bus_message_type_filter_mask(types, 2),
              create_can_bus_message_type_filter_mask({
                  static_cast<uint8_t>(BaroMeasurementMessage::MESSAGE_TYPE),
                  static_cast<uint8_t>(DataTransferMessage::MESSAGE_TYPE),
              }));

    // An empty list still lets reset and unix time through.
    uint32_t empty_mask = create_can_bus_message_type_filter_mask(nullptr, 0);
    EXPECT_EQ(CanBusExtendedId::create(0, ResetMessage::MESSAGE_TYPE, 0, 0) & empty_mask, 0u);
    EXPECT_EQ(CanBusExtendedId::create(0, UnixTimeMessage::MESSAGE_TYPE, 0, 0) & empty_mask, 0u);

    // The mask only ever touches the message type field, so priority, node type
    // and node id can never cause a rejection.
    EXPECT_EQ(CanBusExtendedId::create(7, 0, 0x3F, 0xFFF) & empty_mask, 0u);
}

// Mirrors node_types.rs. The payload's own node_type comes from here.
TEST(NodeTypesTest, MatchesRust) {
    using namespace firmware_common::can_bus;
    EXPECT_EQ(VOID_LAKE_NODE_TYPE, 5);
    EXPECT_EQ(AMP_NODE_TYPE, 10);
    EXPECT_EQ(ICARUS_NODE_TYPE, 15);
    EXPECT_EQ(PAYLOAD_SDRM_NODE_TYPE, 20);
    EXPECT_EQ(OZYS_NODE_TYPE, 25);
    EXPECT_EQ(BULKHEAD_NODE_TYPE, 30);
    EXPECT_EQ(AERO_RUST_NODE_TYPE, 50);
}

TEST(CanNodeIdTest, FromSerialNumber) {
    using namespace firmware_common::can_bus;

    // CRC-16/IBM-3740 check value, guards the shared can_crc16 helper.
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(can_crc16(check, sizeof(check)), 0x29B1);

    // A 12 byte STM32 style UID: crc is 0x1577, so the node id is the low 12 bits.
    const uint8_t uid[] = {0x53, 0x00, 0x36, 0x00, 0x0A, 0x50, 0x53, 0x53, 0x30, 0x39, 0x36, 0x37};
    EXPECT_EQ(can_crc16(uid, sizeof(uid)), 0x1577);
    EXPECT_EQ(can_node_id_from_serial_number(uid, sizeof(uid)), 0x577);

    // Node ids always fit the 12 bit field of the extended id.
    EXPECT_EQ(can_node_id_from_serial_number(uid, sizeof(uid)) & ~0xFFF, 0u);
}

// An absent strain gauge channel is carried as NaN, same as Rust's Option<f32>.

TEST(OzysMeasurementTest, AbsentChannelsAreNaN) {
    using namespace firmware_common::can_bus;

    auto msg = OzysMeasurementMessage::new_msg(std::nullopt, 1.5f, std::nullopt, -2.25f);
    EXPECT_FALSE(msg.sg_1().has_value());
    EXPECT_EQ(msg.sg_2().value(), 1.5f);
    EXPECT_FALSE(msg.sg_3().has_value());
    EXPECT_EQ(msg.sg_4().value(), -2.25f);

    uint8_t buffer[OzysMeasurementMessage::SIZE_BYTES];
    msg.serialize(buffer);
    auto round_tripped = OzysMeasurementMessage::deserialize(buffer);
    EXPECT_FALSE(round_tripped.sg_1().has_value());
    EXPECT_EQ(round_tripped.sg_2().value(), 1.5f);
}

TEST(CanBusMultiFrameDecoderTest, LRUDiscard) {
    firmware_common::can_bus::CanBusMultiFrameDecoder decoder;
    
    // Fill up all 8 state machines with first frames of different IDs
    for (int i = 0; i < 8; ++i) {
        auto msg = firmware_common::can_bus::CustomPayloadStatusMessage::new_unavailable(); // 20 bytes
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(
            1, firmware_common::can_bus::CustomPayloadStatusMessage::MESSAGE_TYPE, 1, i);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, static_cast<uint64_t>(1000 + i));
        EXPECT_FALSE(decoded.has_value());
    }

    // Now send a 9th ID, it should discard the one with timestamp 1000 (i=0)
    {
        auto msg = firmware_common::can_bus::CustomPayloadStatusMessage::new_unavailable();
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(
            1, firmware_common::can_bus::CustomPayloadStatusMessage::MESSAGE_TYPE, 1, 100);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 2000);
        EXPECT_FALSE(decoded.has_value());
    }

    // If we now send the second frame for ID 0, it should fail/restart because it was discarded
    {
        auto msg = firmware_common::can_bus::CustomPayloadStatusMessage::new_unavailable();
        firmware_common::can_bus::CanBusMultiFrameEncoder encoder(msg);
        encoder.next(); // skip first
        auto frame_data = encoder.next();
        uint32_t id = firmware_common::can_bus::CanBusExtendedId::create(
            1, firmware_common::can_bus::CustomPayloadStatusMessage::MESSAGE_TYPE, 1, 0);
        auto decoded = decoder.process_frame(id, frame_data.data, frame_data.len, 3000);
        EXPECT_FALSE(decoded.has_value());
    }
}

