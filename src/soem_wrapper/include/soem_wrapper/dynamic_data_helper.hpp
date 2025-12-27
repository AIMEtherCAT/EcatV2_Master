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

#include "soem_wrapper/utils/io_utils.hpp"

#include "fmt/core.h"
#include "fmt/format.h"
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/write_dshot.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/write_lk_motor_openloop_control.hpp"
#include "custom_msgs/msg/write_lk_motor_speed_control_with_torque_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_torque_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/write_on_board_pwm.hpp"
#include "custom_msgs/msg/write_external_pwm.hpp"
#include "custom_msgs/msg/read_ms5837_ba30.hpp"
#include "custom_msgs/msg/read_adc.hpp"
#include "custom_msgs/msg/read_canpmu.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_dm_motor_speed_control.hpp"

namespace aim::utils::dynamic_data {
    using namespace aim::io::little_endian;

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
        rclcpp::Subscription<custom_msgs::msg::WriteOnBoardPWM>::SharedPtr,
        rclcpp::Subscription<custom_msgs::msg::WriteExternalPWM>::SharedPtr,
        rclcpp::Publisher<custom_msgs::msg::ReadMS5837BA30>::SharedPtr,
        rclcpp::Publisher<custom_msgs::msg::ReadADC>::SharedPtr,
        rclcpp::Publisher<custom_msgs::msg::ReadCANPMU>::SharedPtr,
        rclcpp::Publisher<custom_msgs::msg::ReadDmMotor>::SharedPtr,
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    >;

    struct BuildBufResultT {
        uint8_t *buf;
        int len;
    };

    // used to check if a Variant type have reset() method
    // if so it means it is a publisher/subscriber
    // and it needs to be destroyed before exit
    // ReSharper disable once CppTemplateParameterNeverUsed
    template<typename T>
    struct is_publisher_or_subscriber : std::false_type {
    };

    template<typename U>
    struct is_publisher_or_subscriber<std::shared_ptr<U> > : std::true_type {
    };

    class DynamicStruct {
    public:
        DynamicStruct(std::initializer_list<std::pair<std::string, Variant> > init);

        void load_initial_value_from_config(const std::string &filepath);

        void parse_map(const std::string &path, const YAML::Node &node);

        void set(const std::string &key, Variant value);

        Variant get(const std::string &key) const;

        bool has(const std::string &key) const;

        void remove(const std::string &key);

        /**
         * copy all values that matches the given full name into a buffer
         * this method is NOT thread-safe
         *
         * @param prefix universal given prefix
         * @param names names
         * @return buffer with corresponding values
         */
        BuildBufResultT build_buf(const std::string &prefix, const std::vector<std::string> &names);

        /**
         * destroy all publisher/subscribers
         * before exit
         */
        void reset_and_remove_publishers();

    private:
        std::unordered_map<std::string, Variant> fields_{};
        uint8_t temp_buf_[1024]{};
        int offset_ = 0;
    };

    template<typename T>
    T get_field_as(const DynamicStruct &data, const uint32_t sn, const uint16_t app_idx, const std::string &suffix,
                   const T &default_value) {
        try {
            return std::get<T>(data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const DynamicStruct &data, const uint32_t sn, const std::string &suffix, const T &default_value) {
        try {
            return std::get<T>(data.get(fmt::format("sn{}_{}", sn, suffix)));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const DynamicStruct &data, const std::string &whole_key, const T &default_value) {
        try {
            return std::get<T>(data.get(whole_key));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const DynamicStruct &data, const uint32_t sn, const uint16_t app_idx, const std::string &suffix) {
        return std::get<T>(data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
    }

    template<typename T>
    T get_field_as(const DynamicStruct &data, const uint32_t sn, const std::string &suffix) {
        return std::get<T>(data.get(fmt::format("sn{}_{}", sn, suffix)));
    }

    template<typename T>
    T get_field_as(const DynamicStruct &data, const std::string &whole_key) {
        return std::get<T>(data.get(whole_key));
    }

    DynamicStruct *get_dynamic_data();
}

#endif //DYNAMIC_DATA_HELPER_H
