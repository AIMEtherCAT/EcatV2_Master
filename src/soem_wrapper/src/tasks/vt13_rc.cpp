#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace logging;
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
        try {
            custom_msgs_readvt13remotecontrol_shared_msg.header.stamp = slave_device_->get_current_data_stamp();
            auto slave_buf = slave_device_->get_slave_to_master_buf();
            const auto required_len = static_cast<size_t>(pdoread_offset_) + RC_MSG_PKG_LEN;
            if (slave_buf.size() < required_len) {
                RCLCPP_ERROR(*get_data_logger(),
                             "VT13_RC read skipped: need %zu bytes from pdoread_offset=%u, but slave_to_master_buf size is %zu",
                             required_len,
                             pdoread_offset_,
                             slave_buf.size());
                publish_empty_message();
                return;
            }
            // Joystick values, 4*11 = 44 bits, bytes 0-5
            custom_msgs_readvt13remotecontrol_shared_msg.right_joystick_x = -(float)(((slave_buf[pdoread_offset_ + 0] 
                | slave_buf[pdoread_offset_ + 1] << 8) & 0x07ff) - 1024) * RC_JOYSTICK_CONV_FACTOR;
            custom_msgs_readvt13remotecontrol_shared_msg.right_joystick_y = (float)(((slave_buf[pdoread_offset_ + 1] >> 3 
                | slave_buf[pdoread_offset_ + 2] << 5) & 0x07ff) - 1024) * RC_JOYSTICK_CONV_FACTOR;
            custom_msgs_readvt13remotecontrol_shared_msg.left_joystick_y = (float)(((slave_buf[pdoread_offset_ + 2] >> 6 
                | slave_buf[pdoread_offset_ + 3] << 2 | slave_buf[pdoread_offset_ + 4] << 10) & 0x07ff) - 1024) * RC_JOYSTICK_CONV_FACTOR;
            custom_msgs_readvt13remotecontrol_shared_msg.left_joystick_x = -(float)(((slave_buf[pdoread_offset_ + 4] >> 1
                | slave_buf[pdoread_offset_ + 5] << 7) & 0x07ff) - 1024) * RC_JOYSTICK_CONV_FACTOR;

            // lower 4 bits in byte 5
            custom_msgs_readvt13remotecontrol_shared_msg.gear_switching     = (slave_buf[pdoread_offset_ + 5] >> 4) & 0x03;
            custom_msgs_readvt13remotecontrol_shared_msg.pause_button       = (slave_buf[pdoread_offset_ + 5] >> 6) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.left_custom_button = (slave_buf[pdoread_offset_ + 5] >> 7) & 0x01;
            // byte 6
            custom_msgs_readvt13remotecontrol_shared_msg.right_custom_button = slave_buf[pdoread_offset_ + 6] & 0x01;
            // thumb wheel, 11 bits, byte 6-7
            custom_msgs_readvt13remotecontrol_shared_msg.thumb_wheel = -(float)(
                ((((int16_t)slave_buf[pdoread_offset_ + 6] >> 1) | (int16_t)slave_buf[pdoread_offset_ + 7] << 7)  & 0x07ff) - 1024)
                    * RC_WHEEL_CONV_FACTOR;
            // trigger, mouse and keyboard fields follow the protocol bit offsets after the 2-byte header
            custom_msgs_readvt13remotecontrol_shared_msg.trigger = (slave_buf[pdoread_offset_ + 7] >> 4) & 0x01;

            custom_msgs_readvt13remotecontrol_shared_msg.mouse_x_axis = (float)(slave_buf[pdoread_offset_ + 8] | slave_buf[pdoread_offset_ + 9] << 8) / (float)INT16_MAX;
            custom_msgs_readvt13remotecontrol_shared_msg.mouse_y_axis = (float)(slave_buf[pdoread_offset_ + 10] | slave_buf[pdoread_offset_ + 11] << 8) / (float)INT16_MAX;
            custom_msgs_readvt13remotecontrol_shared_msg.mouse_wheel = (float)(slave_buf[pdoread_offset_ + 12] | slave_buf[pdoread_offset_ + 13] << 8) / (float)INT16_MAX;

            custom_msgs_readvt13remotecontrol_shared_msg.mouse_lb = slave_buf[pdoread_offset_ + 14] & 0x03;
            custom_msgs_readvt13remotecontrol_shared_msg.mouse_rb = (slave_buf[pdoread_offset_ + 14] >> 2) & 0x03;
            custom_msgs_readvt13remotecontrol_shared_msg.mouse_mb = (slave_buf[pdoread_offset_ + 14] >> 4) & 0x03;

            custom_msgs_readvt13remotecontrol_shared_msg.key_w = slave_buf[pdoread_offset_ + 15] & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_s = (slave_buf[pdoread_offset_ + 15] >> 1) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_a = (slave_buf[pdoread_offset_ + 15] >> 2) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_d = (slave_buf[pdoread_offset_ + 15] >> 3) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_shift = (slave_buf[pdoread_offset_ + 15] >> 4) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_ctrl = (slave_buf[pdoread_offset_ + 15] >> 5) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_q = (slave_buf[pdoread_offset_ + 15] >> 6) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_e = (slave_buf[pdoread_offset_ + 15] >> 7) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_r = slave_buf[pdoread_offset_ + 16] & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_f = (slave_buf[pdoread_offset_ + 16] >> 1) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_g = (slave_buf[pdoread_offset_ + 16] >> 2) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_z = (slave_buf[pdoread_offset_ + 16] >> 3) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_x = (slave_buf[pdoread_offset_ + 16] >> 4) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_c = (slave_buf[pdoread_offset_ + 16] >> 5) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_v = (slave_buf[pdoread_offset_ + 16] >> 6) & 0x01;
            custom_msgs_readvt13remotecontrol_shared_msg.key_b = (slave_buf[pdoread_offset_ + 16] >> 7) & 0x01;
            publisher_->publish(custom_msgs_readvt13remotecontrol_shared_msg);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(*get_data_logger(), "VT13_RC read failed: %s", e.what());
            publish_empty_message();
        } catch (...) {
            RCLCPP_ERROR(*get_data_logger(), "VT13_RC read failed with unknown exception");
            publish_empty_message();
        }
    }
}