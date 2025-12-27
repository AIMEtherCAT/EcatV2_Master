//
// Created by hang on 12/26/25.
//

#ifndef BUILD_LOGGER_H
#define BUILD_LOGGER_H

#include "rclcpp/rclcpp.hpp"

namespace aim::ecat::logging {
    rclcpp::Logger *get_sys_logger();

    rclcpp::Logger *get_cfg_logger();

    rclcpp::Logger *get_data_logger();

    rclcpp::Logger *get_wrapper_logger();

    rclcpp::Logger *get_health_checker_logger();
}

#endif //BUILD_LOGGER_H
