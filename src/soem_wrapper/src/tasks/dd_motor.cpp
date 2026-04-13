//
// Created by hang on 2026/4/14.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"


namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::config;
    using namespace dd_motor;

    custom_msgs::msg::ReadDDMotor DD_MOTOR::custom_msgs_readddmotor_shared_msg;

    void DD_MOTOR::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id, const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                      {
                                                                          "can_baudrate",
                                                                          "connection_lost_write_action",
                                                                          "control_period",
                                                                          "can_packet_id", "motor1_can_id",
                                                                          "motor2_can_id", "motor3_can_id",
                                                                          "motor4_can_id",
                                                                          "can_inst"
                                                                      });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        for (int i = 1; i <= 4; i++) {
            if (const std::string motor_arg_prefix = fmt::format("{}sdowrite_motor{}_", prefix, i);
                get_field_as<uint32_t>(
                    *get_configuration_data(),
                    fmt::format("{}can_id", motor_arg_prefix)) > 0) {
                is_motor_enabled[i - 1] = true;
                auto [sdo_buf_ctrl_current, sdo_len_ctrl_current] = get_configuration_data()->build_buf(
                    motor_arg_prefix, {"control_type"});
                memcpy(buf + *offset, sdo_buf_ctrl_current, sdo_len_ctrl_current);
                *offset += sdo_len_ctrl_current;
            }
        }

        load_slave_info(slave_id, prefix);

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadDDMotor>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );

        subscriber_ = get_node()->create_subscription<custom_msgs::msg::WriteDDMotor>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}sub_topic", prefix)),
            rclcpp::SensorDataQoS(),
            std::bind(&DD_MOTOR::on_command, this, std::placeholders::_1)
        );
    }

    void DD_MOTOR::publish_empty_message() {
        custom_msgs_readddmotor_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readddmotor_shared_msg.motor1_online = 0;
        custom_msgs_readddmotor_shared_msg.motor1_ecd = 0;
        custom_msgs_readddmotor_shared_msg.motor1_rpm = 0;
        custom_msgs_readddmotor_shared_msg.motor1_current = 0;
        custom_msgs_readddmotor_shared_msg.motor1_mode = 0;
        custom_msgs_readddmotor_shared_msg.motor1_error_code = 0;

        custom_msgs_readddmotor_shared_msg.motor2_online = 0;
        custom_msgs_readddmotor_shared_msg.motor2_ecd = 0;
        custom_msgs_readddmotor_shared_msg.motor2_rpm = 0;
        custom_msgs_readddmotor_shared_msg.motor2_current = 0;
        custom_msgs_readddmotor_shared_msg.motor2_mode = 0;
        custom_msgs_readddmotor_shared_msg.motor2_error_code = 0;

        custom_msgs_readddmotor_shared_msg.motor3_online = 0;
        custom_msgs_readddmotor_shared_msg.motor3_ecd = 0;
        custom_msgs_readddmotor_shared_msg.motor3_rpm = 0;
        custom_msgs_readddmotor_shared_msg.motor3_current = 0;
        custom_msgs_readddmotor_shared_msg.motor3_mode = 0;
        custom_msgs_readddmotor_shared_msg.motor3_error_code = 0;

        custom_msgs_readddmotor_shared_msg.motor4_online = 0;
        custom_msgs_readddmotor_shared_msg.motor4_ecd = 0;
        custom_msgs_readddmotor_shared_msg.motor4_rpm = 0;
        custom_msgs_readddmotor_shared_msg.motor4_current = 0;
        custom_msgs_readddmotor_shared_msg.motor4_mode = 0;
        custom_msgs_readddmotor_shared_msg.motor4_error_code = 0;

        publisher_->publish(custom_msgs_readddmotor_shared_msg);
    }

    void DD_MOTOR::init_value() {
        int offset = pdowrite_offset_;

        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);

        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);

        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);

        write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void DD_MOTOR::read() {
        custom_msgs_readddmotor_shared_msg.header.stamp = slave_device_->get_current_data_stamp();
        shared_offset_ = pdoread_offset_;

        if (is_motor_enabled[0]) {
            custom_msgs_readddmotor_shared_msg.motor1_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor1_ecd = read_uint16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor1_rpm = static_cast<float>(read_int16(
                                                                slave_device_->get_slave_to_master_buf().data(),
                                                                &shared_offset_)) / 100.f;
            custom_msgs_readddmotor_shared_msg.motor1_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor1_mode = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor1_error_code = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
        } else {
            custom_msgs_readddmotor_shared_msg.motor1_online = 0;
            custom_msgs_readddmotor_shared_msg.motor1_ecd = 0;
            custom_msgs_readddmotor_shared_msg.motor1_rpm = 0;
            custom_msgs_readddmotor_shared_msg.motor1_current = 0;
            custom_msgs_readddmotor_shared_msg.motor1_mode = 0;
            custom_msgs_readddmotor_shared_msg.motor1_error_code = 0;
        }

        if (is_motor_enabled[1]) {
            custom_msgs_readddmotor_shared_msg.motor2_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor2_ecd = read_uint16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor2_rpm = static_cast<float>(read_int16(
                                                                slave_device_->get_slave_to_master_buf().data(),
                                                                &shared_offset_)) / 100.f;
            custom_msgs_readddmotor_shared_msg.motor2_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor2_mode = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor2_error_code = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
        } else {
            custom_msgs_readddmotor_shared_msg.motor2_online = 0;
            custom_msgs_readddmotor_shared_msg.motor2_ecd = 0;
            custom_msgs_readddmotor_shared_msg.motor2_rpm = 0;
            custom_msgs_readddmotor_shared_msg.motor2_current = 0;
            custom_msgs_readddmotor_shared_msg.motor2_mode = 0;
            custom_msgs_readddmotor_shared_msg.motor2_error_code = 0;
        }

        if (is_motor_enabled[2]) {
            custom_msgs_readddmotor_shared_msg.motor3_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor3_ecd = read_uint16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor3_rpm = static_cast<float>(read_int16(
                                                                slave_device_->get_slave_to_master_buf().data(),
                                                                &shared_offset_)) / 100.f;
            custom_msgs_readddmotor_shared_msg.motor3_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor3_mode = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor3_error_code = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
        } else {
            custom_msgs_readddmotor_shared_msg.motor3_online = 0;
            custom_msgs_readddmotor_shared_msg.motor3_ecd = 0;
            custom_msgs_readddmotor_shared_msg.motor3_rpm = 0;
            custom_msgs_readddmotor_shared_msg.motor3_current = 0;
            custom_msgs_readddmotor_shared_msg.motor3_mode = 0;
            custom_msgs_readddmotor_shared_msg.motor3_error_code = 0;
        }

        if (is_motor_enabled[3]) {
            custom_msgs_readddmotor_shared_msg.motor4_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor4_ecd = read_uint16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor4_rpm = static_cast<float>(read_int16(
                                                                slave_device_->get_slave_to_master_buf().data(),
                                                                &shared_offset_)) / 100.f;
            custom_msgs_readddmotor_shared_msg.motor4_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor4_mode = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readddmotor_shared_msg.motor4_error_code = read_uint8(
                slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
        } else {
            custom_msgs_readddmotor_shared_msg.motor4_online = 0;
            custom_msgs_readddmotor_shared_msg.motor4_ecd = 0;
            custom_msgs_readddmotor_shared_msg.motor4_rpm = 0;
            custom_msgs_readddmotor_shared_msg.motor4_current = 0;
            custom_msgs_readddmotor_shared_msg.motor4_mode = 0;
            custom_msgs_readddmotor_shared_msg.motor4_error_code = 0;
        }

        publisher_->publish(custom_msgs_readddmotor_shared_msg);
    }

    void DD_MOTOR::on_command(const custom_msgs::msg::WriteDDMotor::SharedPtr msg) const { // NOLINT
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        if (is_motor_enabled[0]) {
            write_uint8(msg->motor1_enable, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(msg->motor1_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
        if (is_motor_enabled[1]) {
            write_uint8(msg->motor2_enable, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(msg->motor2_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
        if (is_motor_enabled[2]) {
            write_uint8(msg->motor3_enable, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(msg->motor3_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
        if (is_motor_enabled[3]) {
            write_uint8(msg->motor4_enable, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(msg->motor4_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
    }
}
