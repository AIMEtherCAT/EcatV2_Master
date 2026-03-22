#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::config;
    using namespace vt13_rc;

    custom_msgs::msg::ReadVT13RemoteControl VT13_RC::custom_msgs_readvt13remotecontrol_shared_msg;

    void VT13_RC::init_sdo(uint8_t * /* buf */, int * /* offset */, const uint16_t slave_id,
                           const std::string &prefix) {
        load_slave_info(slave_id, prefix);

        custom_msgs_readvt13remotecontrol_shared_msg = custom_msgs::msg::ReadVT13RemoteControl{};
        
        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadVT13RemoteControl>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void VT13_RC::publish_empty_message() {
        custom_msgs_readvt13remotecontrol_shared_msg = custom_msgs::msg::ReadVT13RemoteControl{};
        custom_msgs_readvt13remotecontrol_shared_msg.header.stamp = rclcpp::Clock().now();
        publisher_->publish(custom_msgs_readvt13remotecontrol_shared_msg);
    }

    void VT13_RC::read() {
        // Joystick values, 4*11 = 44 bits, bytes 0-5
        custom_msgs_readvt13remotecontrol_shared_msg.header.stamp = slave_device_->get_current_data_stamp();
        auto slave_buf = slave_device_->get_slave_to_master_buf();
        custom_msgs_readvt13remotecontrol_shared_msg.right_joystick_x = float(
            ((int16_t)slave_buf[pdoread_offset_ + 0] << 3 | (int16_t)slave_buf[pdoread_offset_ + 1] >> 5)
                 - 1024) * RC_JOYSTICK_CONV_FACTOR;
        custom_msgs_readvt13remotecontrol_shared_msg.right_joystick_y = float(
            (((int16_t)slave_buf[pdoread_offset_ + 1] & 0x1F) << 6 | (int16_t)slave_buf[pdoread_offset_ + 2] >> 2)
                 - 1024) * RC_JOYSTICK_CONV_FACTOR;
        custom_msgs_readvt13remotecontrol_shared_msg.left_joystick_y = float(
            (((int16_t)slave_buf[pdoread_offset_ + 2] & 0x03) << 9 | (int16_t)slave_buf[pdoread_offset_ + 3] << 1 
                | (int16_t)slave_buf[pdoread_offset_ + 4] >> 7) - 1024) * RC_JOYSTICK_CONV_FACTOR;
        custom_msgs_readvt13remotecontrol_shared_msg.left_joystick_x = float(
            (((int16_t)slave_buf[pdoread_offset_ + 4] & 0x7F) << 4 | (int16_t)slave_buf[pdoread_offset_ + 5] >> 4)
                 - 1024) * RC_JOYSTICK_CONV_FACTOR;
        // lower 4 bits in byte 5
        custom_msgs_readvt13remotecontrol_shared_msg.gear_switching     = (slave_buf[pdoread_offset_ + 5] & 0x0C) >> 2;
        custom_msgs_readvt13remotecontrol_shared_msg.pause_button       = (slave_buf[pdoread_offset_ + 5] & 0x02) >> 1;
        custom_msgs_readvt13remotecontrol_shared_msg.left_custom_button = (slave_buf[pdoread_offset_ + 5] & 0x01);
        // byte 6
        custom_msgs_readvt13remotecontrol_shared_msg.right_custom_button = (slave_buf[pdoread_offset_ + 6] & 0x80) >> 7;
        // thumb wheel, 11 bits, byte 6-7
        custom_msgs_readvt13remotecontrol_shared_msg.thumb_wheel = float(
            (((int16_t)slave_buf[pdoread_offset_ + 6] & 0x7F) << 4 | (int16_t)slave_buf[pdoread_offset_ + 7] >> 4) - 1024) 
                * RC_WHEEL_CONV_FACTOR;
        // byte 7 bit 3, trigger
        custom_msgs_readvt13remotecontrol_shared_msg.trigger = (slave_buf[pdoread_offset_ + 7] & 0x08) >> 3;
        // mouse, byte 8-14
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_x_axis = float(
            (float)((int16_t)slave_buf[pdoread_offset_ + 8] << 8 | (int16_t)slave_buf[pdoread_offset_ + 9])) * (1.0f/(float)INT16_MAX);
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_y_axis = float(
            (float)((int16_t)slave_buf[pdoread_offset_ + 10] << 8 | (int16_t)slave_buf[pdoread_offset_ + 11])) * (1.0f/(float)INT16_MAX);
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_wheel = float(
            (float)((int16_t)slave_buf[pdoread_offset_ + 12] << 8 | (int16_t)slave_buf[pdoread_offset_ + 13])) * (1.0f/(float)INT16_MAX);
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_lb = (slave_buf[pdoread_offset_ + 14] & 0xC0) >> 6;
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_rb = (slave_buf[pdoread_offset_ + 14] & 0x30) >> 4;
        custom_msgs_readvt13remotecontrol_shared_msg.mouse_mb = (slave_buf[pdoread_offset_ + 14] & 0x0C) >> 2;
        // keyboard, byte 15-16
        custom_msgs_readvt13remotecontrol_shared_msg.key_w = (slave_buf[pdoread_offset_ + 15] & 0x80) >> 7;
        custom_msgs_readvt13remotecontrol_shared_msg.key_s = (slave_buf[pdoread_offset_ + 15] & 0x40) >> 6;
        custom_msgs_readvt13remotecontrol_shared_msg.key_a = (slave_buf[pdoread_offset_ + 15] & 0x20) >> 5;
        custom_msgs_readvt13remotecontrol_shared_msg.key_d = (slave_buf[pdoread_offset_ + 15] & 0x10) >> 4;
        custom_msgs_readvt13remotecontrol_shared_msg.key_shift = (slave_buf[pdoread_offset_ + 15] & 0x08) >> 3;
        custom_msgs_readvt13remotecontrol_shared_msg.key_ctrl = (slave_buf[pdoread_offset_ + 15] & 0x04) >> 2;
        custom_msgs_readvt13remotecontrol_shared_msg.key_q = (slave_buf[pdoread_offset_ + 15] & 0x02) >> 1;
        custom_msgs_readvt13remotecontrol_shared_msg.key_e = (slave_buf[pdoread_offset_ + 15] & 0x01);
        custom_msgs_readvt13remotecontrol_shared_msg.key_r = (slave_buf[pdoread_offset_ + 16] & 0x80) >> 7;
        custom_msgs_readvt13remotecontrol_shared_msg.key_f = (slave_buf[pdoread_offset_ + 16] & 0x40) >> 6;
        custom_msgs_readvt13remotecontrol_shared_msg.key_g = (slave_buf[pdoread_offset_ + 16] & 0x20) >> 5;
        custom_msgs_readvt13remotecontrol_shared_msg.key_z = (slave_buf[pdoread_offset_ + 16] & 0x10) >> 4;
        custom_msgs_readvt13remotecontrol_shared_msg.key_x = (slave_buf[pdoread_offset_ + 16] & 0x08) >> 3;
        custom_msgs_readvt13remotecontrol_shared_msg.key_c = (slave_buf[pdoread_offset_ + 16] & 0x04) >> 2;
        custom_msgs_readvt13remotecontrol_shared_msg.key_v = (slave_buf[pdoread_offset_ + 16] & 0x02) >> 1;
        custom_msgs_readvt13remotecontrol_shared_msg.key_b = (slave_buf[pdoread_offset_ + 16] & 0x01);
        publisher_->publish(custom_msgs_readvt13remotecontrol_shared_msg);
    }
}