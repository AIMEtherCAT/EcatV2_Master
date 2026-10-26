//
// Created by hang on 26-4-9.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::config;
    using namespace pmu_uavcan;

    void PMU_CAN::init_sdo(uint8_t * /* buf */, int * /* offset */, const uint16_t slave_id,
                           const std::string &prefix) {
        load_slave_info(slave_id, prefix);

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadCANPMU>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void PMU_CAN::publish_empty_message() {
        custom_msgs_readcanpmu_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readcanpmu_shared_msg.online = 0;
        custom_msgs_readcanpmu_shared_msg.current = 0;
        custom_msgs_readcanpmu_shared_msg.temperature = 0;
        custom_msgs_readcanpmu_shared_msg.voltage = 0;

        publisher_->publish(custom_msgs_readcanpmu_shared_msg);
    }

    void PMU_CAN::read() {
        custom_msgs_readcanpmu_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;

        shared_offset_ = pdoread_offset_;

        custom_msgs_readcanpmu_shared_msg.online = 0;
        if (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 6]) {
            custom_msgs_readcanpmu_shared_msg.online = 1;
            custom_msgs_readcanpmu_shared_msg.temperature = read_float16(slave_device_->get_slave_to_master_buf().data(), &shared_offset_) - 273.15f;
            custom_msgs_readcanpmu_shared_msg.voltage = read_float16(slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readcanpmu_shared_msg.current = read_float16(slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
        }

        publisher_->publish(custom_msgs_readcanpmu_shared_msg);
    }
}
