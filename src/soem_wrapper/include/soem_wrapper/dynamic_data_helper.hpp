//
// Created by hang on 25-5-4.
//

#ifndef DYNAMIC_DATA_HELPER_H
#define DYNAMIC_DATA_HELPER_H

#include <iostream>
#include <unordered_map>
#include <utility>
#include <variant>
#include <string>
#include <vector>
#include <memory>
#include "utils.hpp"
#include "fmt/core.h"
#include "fmt/format.h"

#include "yaml-cpp/yaml.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <chrono>
// ReSharper disable once CppUnusedIncludeDirective
#include <functional>

// this header files will be used by dynamic_data_helper.cpp main func
// ReSharper disable once CppUnusedIncludeDirective
#include "defs/packet_defs.h"

#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/write_dshot.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include "custom_msgs/msg/write_lk_motor_openloop_control.hpp"
#include "custom_msgs/msg/write_lk_motor_speed_control_with_torque_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_torque_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control_with_speed_limit.hpp"

#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/write_vanilla_pwm.hpp"
#include "custom_msgs/msg/write_external_pwm.hpp"
#include "custom_msgs/msg/read_ms5837_ba30.hpp"
#include "custom_msgs/msg/read_adc.hpp"
#include "custom_msgs/msg/read_canpmu.hpp"

#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_dm_motor_speed_control.hpp"

using Variant = std::variant<
    uint8_t,
    uint16_t,
    int8_t,
    int16_t,
    uint32_t,
    int32_t,
    float,
    std::string,
    bool,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadLkMotor>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadDJIRC>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadDJIMotor>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadSBUSRC>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteDSHOT>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteDmMotorSpeedControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorOpenloopControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorTorqueControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteDJIMotor>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteVanillaPWM>::SharedPtr,
    rclcpp::Subscription<custom_msgs::msg::WriteExternalPWM>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadMS5837BA30>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadADC>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadCANPMU>::SharedPtr,
    rclcpp::Publisher<custom_msgs::msg::ReadDmMotor>::SharedPtr,
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
>;

// used to check if a Variant type have reset() method
// if so it means it is a publisher/subscriber
// and it needs to be destroyed before exit
// ReSharper disable once CppTemplateParameterNeverUsed
template<typename T>
struct is_shared_ptr : std::false_type {
};

template<typename U>
struct is_shared_ptr<std::shared_ptr<U> > : std::true_type {
};

class DynamicStruct {
public:
    DynamicStruct(std::initializer_list<std::pair<std::string, Variant> > init) {
        for (const auto &[key, value]: init) {
            fields[key] = value;
        }
        memset(temp_buf, 0, 1024);
    }

    void load_initial_value_from_config(const std::string &filepath);

    void parse_map(const std::string &path, const YAML::Node &node);

    void
    set(const std::string &key, Variant value) {
        fields[key] = std::move(value);
    }

    Variant
    get(const std::string &key) const {
        if (const auto it = fields.find(key);
            it != fields.end())
            return it->second;
        throw std::runtime_error("get Key not found: " + key);
    }

    Variant
    get(const std::string &key, const int idx) const {
        if (const auto it = fields.find(key);
            it != fields.end())
            return it->second;
        throw std::runtime_error("get Key not found: " + key + std::to_string(idx));
    }

    bool
    has(const std::string &key) const {
        return fields.find(key) != fields.end();
    }

    void
    remove(const std::string &key) {
        if (const auto it = fields.find(key);
            it != fields.end()) {
            fields.erase(it);
        } else {
            throw std::runtime_error("del Key not found: " + key);
        }
    }

    /**
     * copy all values that matches the given full name into a buffer
     * @param prefix universal given prefix
     * @param names names
     * @return buffer with corresponding values
     */
    uint8_t *
    build_buf(const std::string &prefix, const std::vector<std::string> &names) {
        offset = 0;
        memset(temp_buf, 0, sizeof(temp_buf));

        for (const std::string &name: names) {
            std::visit(
                [&](auto &&arg) {
                    using T = std::decay_t<decltype(arg)>;

                    if constexpr (std::is_same_v<T, uint8_t>) {
                        write_uint8(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, int8_t>) {
                        write_int8(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, uint16_t>) {
                        write_uint16(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, int16_t>) {
                        write_int16(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, uint32_t>) {
                        write_uint32(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, int32_t>) {
                        write_int32(arg, temp_buf, &offset);
                    } else if constexpr (std::is_same_v<T, float>) {
                        write_float(arg, temp_buf, &offset);
                    }
                },
                get(fmt::format("{}{}", prefix, name))
            );
        }

        return temp_buf;
    }

    /**
     * destroy all publisher/subscribers
     * before exit
     */
    void
    reset_and_remove_publishers() {
        std::vector<std::string> keys_to_remove;

        for (auto &[key, value]: fields) {
            // all pub/sub inst ends with _inst
            if (key.size() >= 5 && key.compare(key.size() - 5, 5, "_inst") == 0) {
                std::visit(
                    [&](auto &inst) {
                        using T = std::decay_t<decltype(inst)>;
                        if constexpr (is_shared_ptr<T>::value) {
                            if (inst) {
                                inst.reset();
                                keys_to_remove.push_back(key);
                            }
                        }
                    },
                    value
                );
            }
        }

        for (const auto &key: keys_to_remove) {
            fields.erase(key);
        }
    }

private:
    std::unordered_map<std::string, Variant> fields{};
    uint8_t temp_buf[1024]{};
    int offset = 0;
};

#endif //DYNAMIC_DATA_HELPER_H
