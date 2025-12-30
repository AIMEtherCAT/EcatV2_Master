//
// Created by hang on 12/30/25.
//
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat::task {
    using namespace utils::config;
    
    void TaskWrapper::load_slave_info(const uint16_t slave_id, const std::string &prefix) {
        slave_device_ = get_slave_device(slave_id);

        pdoread_offset_ = get_field_as<uint16_t>(
            *get_configuration_data(),
            fmt::format("{}pdoread_offset", prefix), 0);

        pdowrite_offset_ = get_field_as<uint16_t>(
            *get_configuration_data(),
            fmt::format("{}pdowrite_offset", prefix), 0);

        connection_lost_read_action_ = get_field_as<uint8_t>(
            *get_configuration_data(),
            fmt::format("{}conf_connection_lost_read_action", prefix),
            0);

        connection_lost_write_action_ = get_field_as<uint8_t>(
            *get_configuration_data(),
            fmt::format("{}sdowrite_connection_lost_write_action", prefix),
            0);
    }

    void TaskWrapper::init_sdo(uint8_t */* buf */, int */* offset */,
                              const uint16_t /* slave_id */, const std::string &prefix) {
        RCLCPP_ERROR(*logging::get_data_logger(),
                     "init_sdo was not overridden and got called for the task: %s, prefix: %s",
                     type_name_.c_str(),
                     prefix.c_str());
    }

    void TaskWrapper::init_value() {
        RCLCPP_ERROR(*logging::get_data_logger(),
                     "init_value was not overridden and got called for the task: %s",
                     type_name_.c_str());
    }

    void TaskWrapper::read() {
        RCLCPP_ERROR(*logging::get_data_logger(),
                     "read was not overridden and got called for the task: %s",
                     type_name_.c_str());
    }

    void TaskWrapper::publish_empty_message() {
        RCLCPP_ERROR(*logging::get_data_logger(),
                     "publish_empty_message was not overridden and got called for the task: %s",
                     type_name_.c_str());
    }

    void TaskWrapper::cleanup() {
        RCLCPP_ERROR(*logging::get_data_logger(),
                     "cleanup was not overridden and got called for the task: %s",
                     type_name_.c_str());
    }
}