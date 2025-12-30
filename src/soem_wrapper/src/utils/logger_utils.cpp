//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat::logging {
    rclcpp::Logger sys_logger = rclcpp::get_logger("Ecat_SYS");
    rclcpp::Logger cfg_logger = rclcpp::get_logger("Ecat_CFG");
    rclcpp::Logger data_logger = rclcpp::get_logger("Ecat_DATA");
    rclcpp::Logger wrapper_logger = rclcpp::get_logger("Ecat_WRAPPER");
    rclcpp::Logger health_checker_logger = rclcpp::get_logger("Ecat_HEALTH_CHECKER");

    rclcpp::Logger *get_sys_logger() {
        return &sys_logger;
    }

    rclcpp::Logger *get_cfg_logger() {
        return &cfg_logger;
    }

    rclcpp::Logger *get_data_logger() {
        return &data_logger;
    }

    rclcpp::Logger *get_wrapper_logger() {
        return &wrapper_logger;
    }

    rclcpp::Logger *get_health_checker_logger() {
        return &health_checker_logger;
    }
}