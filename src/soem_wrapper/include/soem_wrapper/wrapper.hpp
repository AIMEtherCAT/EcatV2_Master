//
// Created by hang on 12/26/25.
//

#ifndef BUILD_SOEM_WRAPPER_H
#define BUILD_SOEM_WRAPPER_H

#include "rclcpp/rclcpp.hpp"

#include "osal.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"

namespace aim::ecat {
    struct SlaveDeviceT {
        ec_slavet *slave = nullptr;
        char sw_rev_str[4];
        int sw_rev = 0;
        uint8_t sw_rev_check_passed = 0;
        uint32_t sn = 0;
        uint8_t recover_rejected = 0;
        uint8_t device_type = 0;

        uint8_t ready = 0;
        rclcpp::Time last_packet_time{};

        uint8_t master_status = 0;
        std::vector<uint8_t> master_to_slave_buf{};
        // used for connection lost recover
        std::vector<uint8_t> master_to_slave_buf_backup{};
        uint16_t master_to_slave_buf_len = 0;

        uint8_t slave_status = 0;
        std::vector<uint8_t> slave_to_master_buf{};
        uint16_t slave_to_master_buf_len = 0;

        std::vector<uint8_t> arg_buf{};
        uint16_t arg_buf_len = 0;
        int arg_buf_idx = 0;

        uint16_t sending_arg_buf_idx = 1;
        uint8_t arg_sent = 0;

        // ecat conf done flag
        uint8_t conf_done = 0;
        // ros2 pub/sub conf done flag
        uint8_t conf_ros_done = 0;
        uint8_t reconnected_times = 0;

        rclcpp::Time data_stamp{};

        mutable std::mutex mtx;
    };

    SlaveDeviceT *get_slave_devices();

    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave);
}

#endif //BUILD_SOEM_WRAPPER_H
