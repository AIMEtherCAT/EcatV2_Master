//
// Created by hang on 3/14/26.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::config;
    using namespace super_cap;

    custom_msgs::msg::ReadSuperCap SUPER_CAP::custom_msgs_readsupercap_shared_msg;

    void SUPER_CAP::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id, const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                      {
                                                                          "connection_lost_write_action",
                                                                          "can_inst",
                                                                          "chassis_to_cap_id", "cap_to_chassis_id_"
                                                                      });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;
        load_slave_info(slave_id, prefix);

        subscriber_ = get_node()->create_subscription<
            custom_msgs::msg::WriteSuperCap>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}sub_topic", prefix)),
            rclcpp::SensorDataQoS(),
            std::bind(&SUPER_CAP::on_command, this, std::placeholders::_1)
        );

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadSuperCap>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void SUPER_CAP::publish_empty_message() {
        custom_msgs_readsupercap_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readsupercap_shared_msg.online = 0;
        custom_msgs_readsupercap_shared_msg.cap_valid = false;
        custom_msgs_readsupercap_shared_msg.cap_status = 0;
        custom_msgs_readsupercap_shared_msg.cap_remain_percentage = 0;
        custom_msgs_readsupercap_shared_msg.chassis_power = 0;
        custom_msgs_readsupercap_shared_msg.battery_voltage = 0;
        custom_msgs_readsupercap_shared_msg.chassis_only_power = 0;

        publisher_->publish(custom_msgs_readsupercap_shared_msg);
    }

    void SUPER_CAP::read() {
        custom_msgs_readsupercap_shared_msg.header.stamp = slave_device_->get_current_data_stamp();

        int offset = pdoread_offset_;

        custom_msgs_readsupercap_shared_msg.online = read_uint8(slave_device_->get_slave_to_master_buf().data(),
                                                                &offset);
        custom_msgs_readsupercap_shared_msg.cap_valid = read_uint8(slave_device_->get_slave_to_master_buf().data(),
                                                                   &offset);
        custom_msgs_readsupercap_shared_msg.cap_status = read_uint8(slave_device_->get_slave_to_master_buf().data(),
                                                                    &offset);
        custom_msgs_readsupercap_shared_msg.cap_remain_percentage = read_uint8(
            slave_device_->get_slave_to_master_buf().data(),
            &offset);
        custom_msgs_readsupercap_shared_msg.chassis_power = read_uint8(slave_device_->get_slave_to_master_buf().data(),
                                                                       &offset);
        custom_msgs_readsupercap_shared_msg.battery_voltage = read_uint8(
            slave_device_->get_slave_to_master_buf().data(),
            &offset);
        custom_msgs_readsupercap_shared_msg.chassis_only_power = read_uint8(
            slave_device_->get_slave_to_master_buf().data(),
            &offset);

        publisher_->publish(custom_msgs_readsupercap_shared_msg);
    }

    void SUPER_CAP::init_value() {
        int offset = pdowrite_offset_;

        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void SUPER_CAP::on_command(custom_msgs::msg::WriteSuperCap::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->cap_enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(msg->do_charge, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(msg->max_charge_power, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(msg->allow_charge_power, slave_device_->get_master_to_slave_buf().data(), &offset);
    }
}
