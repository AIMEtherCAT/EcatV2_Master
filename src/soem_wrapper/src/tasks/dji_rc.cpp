//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::config;
    using namespace dbus_rc;

    custom_msgs::msg::ReadDJIRC DBUS_RC::custom_msgs_readdjirc_shared_msg;

    void DBUS_RC::init_sdo(uint8_t * /*buf*/, int * /*offset*/, const uint16_t slave_id, const std::string &prefix) {
        load_slave_info(slave_id, prefix);

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadDJIRC>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void DBUS_RC::publish_empty_message() {
        custom_msgs_readdjirc_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readdjirc_shared_msg.right_x = 0;
        custom_msgs_readdjirc_shared_msg.right_y = 0;
        custom_msgs_readdjirc_shared_msg.left_x = 0;
        custom_msgs_readdjirc_shared_msg.left_y = 0;
        custom_msgs_readdjirc_shared_msg.dial = 0;
        custom_msgs_readdjirc_shared_msg.right_switch = 0;
        custom_msgs_readdjirc_shared_msg.left_switch = 0;
        custom_msgs_readdjirc_shared_msg.mouse_x = 0;
        custom_msgs_readdjirc_shared_msg.mouse_y = 0;
        custom_msgs_readdjirc_shared_msg.mouse_left_clicked = 0;
        custom_msgs_readdjirc_shared_msg.mouse_right_clicked = 0;
        custom_msgs_readdjirc_shared_msg.w = 0;
        custom_msgs_readdjirc_shared_msg.s = 0;
        custom_msgs_readdjirc_shared_msg.a = 0;
        custom_msgs_readdjirc_shared_msg.d = 0;
        custom_msgs_readdjirc_shared_msg.shift = 0;
        custom_msgs_readdjirc_shared_msg.ctrl = 0;
        custom_msgs_readdjirc_shared_msg.q = 0;
        custom_msgs_readdjirc_shared_msg.e = 0;
        custom_msgs_readdjirc_shared_msg.r = 0;
        custom_msgs_readdjirc_shared_msg.f = 0;
        custom_msgs_readdjirc_shared_msg.g = 0;
        custom_msgs_readdjirc_shared_msg.z = 0;
        custom_msgs_readdjirc_shared_msg.x = 0;
        custom_msgs_readdjirc_shared_msg.c = 0;
        custom_msgs_readdjirc_shared_msg.v = 0;
        custom_msgs_readdjirc_shared_msg.b = 0;

        custom_msgs_readdjirc_shared_msg.online = 0;
        publisher_->publish(custom_msgs_readdjirc_shared_msg);
    }

    static float get_percentage(const uint16_t raw_data) {
        const float target_spd = static_cast<float>(raw_data - 1024);
        if (target_spd >= -5 && target_spd <= 5) {
            return 0.0f;
        }
        return target_spd / 660.0f;
    }

    void DBUS_RC::read() {
        custom_msgs_readdjirc_shared_msg.header.stamp = slave_device_->get_current_data_stamp();

        if (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 18]) {
            // channel
            custom_msgs_readdjirc_shared_msg.right_x = get_percentage(
                (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 0] | slave_device_->
                 get_slave_to_master_buf()[pdoread_offset_ + 1] << 8) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.right_y = get_percentage(
                (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 1] >> 3 | slave_device_->
                 get_slave_to_master_buf()[pdoread_offset_ + 2] << 5) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.left_x = get_percentage(
                (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 2] >> 6 | slave_device_->
                 get_slave_to_master_buf()[pdoread_offset_ + 3] << 2 | slave_device_->get_slave_to_master_buf()[
                     pdoread_offset_ + 4] << 10) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.left_y = get_percentage(
                (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 4] >> 1 | slave_device_->
                 get_slave_to_master_buf()[pdoread_offset_ + 5] << 7) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.dial = get_percentage(
                (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 16] | slave_device_->
                 get_slave_to_master_buf()[pdoread_offset_ + 17] << 8) & 0x07ff);

            // switcher
            custom_msgs_readdjirc_shared_msg.right_switch =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 5] >> 4 & 0x03;
            custom_msgs_readdjirc_shared_msg.left_switch =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 5] >> 6 & 0x03;

            // mouse
            custom_msgs_readdjirc_shared_msg.mouse_x = static_cast<int16_t>(
                slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 6] | slave_device_->get_slave_to_master_buf()
                [pdoread_offset_ + 7] << 8);
            custom_msgs_readdjirc_shared_msg.mouse_y = static_cast<int16_t>(
                slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 8] | slave_device_->get_slave_to_master_buf()
                [pdoread_offset_ + 9] << 8);
            custom_msgs_readdjirc_shared_msg.mouse_left_clicked = slave_device_->get_slave_to_master_buf()[
                pdoread_offset_ + 12];
            custom_msgs_readdjirc_shared_msg.mouse_right_clicked = slave_device_->get_slave_to_master_buf()[
                pdoread_offset_ + 13];

            // keyboard
            const uint16_t key_value = static_cast<uint16_t>(
                slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 14] | slave_device_->
                get_slave_to_master_buf()[pdoread_offset_ + 15] << 8);
            custom_msgs_readdjirc_shared_msg.w = key_value >> 0 & 0x01;
            custom_msgs_readdjirc_shared_msg.s = key_value >> 1 & 0x01;
            custom_msgs_readdjirc_shared_msg.a = key_value >> 2 & 0x01;
            custom_msgs_readdjirc_shared_msg.d = key_value >> 3 & 0x01;
            custom_msgs_readdjirc_shared_msg.shift = key_value >> 4 & 0x01;
            custom_msgs_readdjirc_shared_msg.ctrl = key_value >> 5 & 0x01;
            custom_msgs_readdjirc_shared_msg.q = key_value >> 6 & 0x01;
            custom_msgs_readdjirc_shared_msg.e = key_value >> 7 & 0x01;
            custom_msgs_readdjirc_shared_msg.r = key_value >> 8 & 0x01;
            custom_msgs_readdjirc_shared_msg.f = key_value >> 9 & 0x01;
            custom_msgs_readdjirc_shared_msg.g = key_value >> 10 & 0x01;
            custom_msgs_readdjirc_shared_msg.z = key_value >> 11 & 0x01;
            custom_msgs_readdjirc_shared_msg.x = key_value >> 12 & 0x01;
            custom_msgs_readdjirc_shared_msg.c = key_value >> 13 & 0x01;
            custom_msgs_readdjirc_shared_msg.v = key_value >> 14 & 0x01;
            custom_msgs_readdjirc_shared_msg.b = key_value >> 15 & 0x01;

            custom_msgs_readdjirc_shared_msg.online = 1;
        } else {
            custom_msgs_readdjirc_shared_msg.online = 0;
        }

        publisher_->publish(custom_msgs_readdjirc_shared_msg);
    }
}
