// ReSharper disable CppCStyleCast
#include "soem_wrapper/dynamic_data_helper.hpp"

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <type_traits>

extern rclcpp::Logger data_logger;

DynamicStruct sdo_data = {};
// = {
//     {"sn2228293_sdo_len", (uint16_t) 29},
//     {"sn2228293_task_count", (uint8_t) 3},
//     {"sn2228293_latency_pub_topic", std::string("/ecat/sn2228293/latency")},
//
//     {"sn2228293_app_1_sdowrite_task_type", (uint8_t)HIPNUC_IMU_CAN_APP_ID},
//     {"sn2228293_app_1_pub_topic", std::string("/imu")},
//     {"sn2228293_app_1_pdoread_offset", (uint16_t) 0},
//
//     {"sn2228293_app_2_sdowrite_task_type", (uint8_t) DJICAN_APP_ID},
//     {"sn2228293_app_2_sdowrite_control_period", (uint16_t) 1},
//     {"sn2228293_app_2_sdowrite_can_packet_id", (uint32_t) 0x1fe},
//     {"sn2228293_app_2_sdowrite_motor1_can_id", (uint32_t) 0x205},
//     {"sn2228293_app_2_sdowrite_motor2_can_id", (uint32_t) 0x206},
//     {"sn2228293_app_2_sdowrite_motor3_can_id", (uint32_t) 0},
//     {"sn2228293_app_2_sdowrite_motor4_can_id", (uint32_t) 0},
//     {"sn2228293_app_2_sdowrite_can_inst", (uint8_t) CAN_PORT_1},
//     {"sn2228293_app_2_sdowrite_motor1_control_type", (uint8_t) DJIMOTOR_CTRL_TYPE_CURRENT},
//     {"sn2228293_app_2_sdowrite_motor2_control_type", (uint8_t) DJIMOTOR_CTRL_TYPE_CURRENT},
//     {"sn2228293_app_2_pub_topic", std::string("/motor_status")},
//     {"sn2228293_app_2_sub_topic", std::string("/motor_out")},
//     {"sn2228293_app_2_pdowrite_offset", (uint16_t) 0},
//     {"sn2228293_app_2_pdoread_offset", (uint16_t) 21},
//
//     {"sn2228293_app_3_sdowrite_task_type", (uint8_t)DJIRC_APP_ID},
//     {"sn2228293_app_3_pub_topic", std::string("/rc")},
//     {"sn2228293_app_3_pdoread_offset", (uint16_t) 35},
// };

void DynamicStruct::parse_map(const std::string& path, const YAML::Node& node)
{
    for (const auto& kv : node)
    {
        auto key = kv.first.as<std::string>();
        const YAML::Node& val = kv.second;
        if (val.IsScalar())
        {
            const std::string insert_key = fmt::format("{}_{}", path, key);
            const std::string& tag = val.Tag();
            if (tag == "!uint8_t") {
                set(insert_key, val.as<uint8_t>());
                RCLCPP_INFO(data_logger, "YAML insert type uint8_t key %s value %u", insert_key.c_str(), val.as<uint8_t>());
            } else if (tag == "!int8_t") {
                set(insert_key, val.as<int8_t>());
                RCLCPP_INFO(data_logger, "YAML insert type int8_t key %s value %d", insert_key.c_str(), val.as<int8_t>());
            } else if (tag == "!uint16_t") {
                set(insert_key, val.as<uint16_t>());
                RCLCPP_INFO(data_logger, "YAML insert type uint16_t key %s value %u", insert_key.c_str(), val.as<uint16_t>());
            } else if (tag == "!int16_t") {
                set(insert_key, val.as<int16_t>());
                RCLCPP_INFO(data_logger, "YAML insert type int16_t key %s value %d", insert_key.c_str(), val.as<int16_t>());
            } else if (tag == "!uint32_t") {
                set(insert_key, val.as<uint32_t>());
                RCLCPP_INFO(data_logger, "YAML insert type uint32_t key %s value %u", insert_key.c_str(), val.as<uint32_t>());
            } else if (tag == "!int32_t") {
                set(insert_key, val.as<int32_t>());
                RCLCPP_INFO(data_logger, "YAML insert type int32_t key %s value %d", insert_key.c_str(), val.as<int32_t>());
            } else if (tag == "!float") {
                set(insert_key, val.as<float>());
                RCLCPP_INFO(data_logger, "YAML insert type float key %s value %f", insert_key.c_str(), val.as<float>());
            } else if (tag == "!std::string") {
                set(insert_key, val.as<std::string>());
                RCLCPP_INFO(data_logger, "YAML insert type std::string key %s value %s", insert_key.c_str(), val.as<std::string>().c_str());
            } else {
                throw std::runtime_error("Unsupported tag: " + tag);
            }
        } else if (val.IsSequence())
        {
            for (auto && slave : val) {
                if (slave.IsMap()) {
                    for (auto it = slave.begin(); it != slave.end(); ++it) {
                        parse_map(fmt::format("{}_{}", path, it->first.as<std::string>()), it->second);
                    }
                }
            }
        }
    }
}


void DynamicStruct::load_initial_value_from_config(const std::string& filepath)
{
    YAML::Node root = YAML::LoadFile(filepath);
    if (root["slaves"] && root["slaves"].IsSequence()) {
        YAML::Node slaves = root["slaves"];
        for (auto && slave : slaves) {
            if (slave.IsMap()) {
                for (auto it = slave.begin(); it != slave.end(); ++it) {
                    parse_map(it->first.as<std::string>(), it->second);
                }
            }
        }
    }
}
