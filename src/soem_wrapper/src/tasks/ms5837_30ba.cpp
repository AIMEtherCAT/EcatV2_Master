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
    using namespace ms5837;

    custom_msgs::msg::ReadMS5837BA30 MS5837_30BA::custom_msgs_readms5837ba30_shared_msg;

    void MS5837_30BA::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id,
                               const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                      {"i2c_id", "osr_id"});
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        load_slave_info(slave_id, prefix);

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadMS5837BA30>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void MS5837_30BA::publish_empty_message() {
        custom_msgs_readms5837ba30_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readms5837ba30_shared_msg.online = 0;
        custom_msgs_readms5837ba30_shared_msg.temperature = 0;
        custom_msgs_readms5837ba30_shared_msg.pressure = 0;

        publisher_->publish(custom_msgs_readms5837ba30_shared_msg);
    }

    void MS5837_30BA::read() {
        custom_msgs_readms5837ba30_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;

        shared_offset_ = pdoread_offset_;

        custom_msgs_readms5837ba30_shared_msg.online = 0;
        if (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 8]) {
            custom_msgs_readms5837ba30_shared_msg.online = 1;
            custom_msgs_readms5837ba30_shared_msg.temperature = read_int32(
                                                                    slave_device_->get_slave_to_master_buf().data(),
                                                                    &shared_offset_) / 100.;
            custom_msgs_readms5837ba30_shared_msg.pressure = read_int32(slave_device_->get_slave_to_master_buf().data(),
                                                                        &shared_offset_) / 10.;
        }

        publisher_->publish(custom_msgs_readms5837ba30_shared_msg);
    }
}
