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
    using namespace lk_motor;

    custom_msgs::msg::ReadLkMotor LK_MOTOR::custom_msgs_readlkmotor_shared_msg;
    custom_msgs::msg::ReadLkMotorMulti LK_MOTOR::custom_msgs_readlkmotormulti_shared_msg;

    void LK_MOTOR::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id, const std::string &prefix) {
        is_broadcast_mode_ = get_field_as<uint8_t>(*get_configuration_data(),
                                                   fmt::format("{}sdowrite_control_type", prefix)) == 0x08;
        if (is_broadcast_mode_) {
            auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                          {
                                                                              "connection_lost_write_action",
                                                                              "control_period",
                                                                              "control_type",
                                                                              "can_inst"
                                                                          });
            memcpy(buf + *offset, sdo_buf, sdo_len);
            *offset += sdo_len;
        } else {
            auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                          {
                                                                              "connection_lost_write_action",
                                                                              "control_period",
                                                                              "control_type",
                                                                              "can_packet_id",
                                                                              "can_inst",
                                                                          });
            memcpy(buf + *offset, sdo_buf, sdo_len);
            *offset += sdo_len;
        }
        load_slave_info(slave_id, prefix);

        switch (get_field_as<uint8_t>(*get_configuration_data(),
                                      fmt::format("{}sdowrite_control_type", prefix))) {
            case LK_CTRL_TYPE_OPENLOOP_CURRENT: {
                subscriber_openloop_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorOpenloopControl>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_openloop_control, this, std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_TORQUE: {
                subscriber_torque_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorTorqueControl>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_torque_control, this, std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_SPEED_WITH_TORQUE_LIMIT: {
                subscriber_speed_with_torque_limit_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_speed_with_torque_limit_control, this, std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_MULTI_ROUND_POSITION: {
                subscriber_multi_round_pos_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_multi_round_pos_control, this, std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                subscriber_multi_round_pos_with_speed_limit_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_multi_round_pos_with_speed_limit_control, this,
                              std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_SINGLE_ROUND_POSITION: {
                subscriber_single_round_pos_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_single_round_pos_control, this, std::placeholders::_1)
                );
                break;
            }

            case LK_CTRL_TYPE_SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                subscriber_single_round_pos_with_speed_limit_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_single_round_pos_with_speed_limit_control, this,
                              std::placeholders::_1)
                );
                break;
            }
            case LK_CTRL_TYPE_BROADCAST_CURRENT: {
                subscriber_broadcast_current_control_ = get_node()->create_subscription<
                    custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>(
                    get_field_as<std::string>(
                        *get_configuration_data(),
                        fmt::format("{}sub_topic", prefix)),
                    rclcpp::SensorDataQoS(),
                    std::bind(&LK_MOTOR::on_command_broadcast_current_control, this,
                              std::placeholders::_1)
                );
                break;
            }
            default: {
            }
        }

        if (is_broadcast_mode_) {
            publisher_multi_motor_ = get_node()->create_publisher<custom_msgs::msg::ReadLkMotorMulti>(
                get_field_as<std::string>(
                    *get_configuration_data(),
                    fmt::format("{}pub_topic", prefix)),
                rclcpp::SensorDataQoS()
            );
        } else {
            publisher_single_motor_ = get_node()->create_publisher<custom_msgs::msg::ReadLkMotor>(
                get_field_as<std::string>(
                    *get_configuration_data(),
                    fmt::format("{}pub_topic", prefix)),
                rclcpp::SensorDataQoS()
            );
        }
    }

    void LK_MOTOR::publish_empty_message() {
        if (is_broadcast_mode_) {
            custom_msgs_readlkmotormulti_shared_msg.header.stamp = rclcpp::Clock().now();

            custom_msgs_readlkmotormulti_shared_msg.motor1_online = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor1_current = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor1_speed = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor1_encoder = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor1_temperature = 0;

            custom_msgs_readlkmotormulti_shared_msg.motor2_online = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor2_current = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor2_speed = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor2_encoder = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor2_temperature = 0;

            custom_msgs_readlkmotormulti_shared_msg.motor3_online = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor3_current = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor3_speed = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor3_encoder = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor3_temperature = 0;

            custom_msgs_readlkmotormulti_shared_msg.motor4_online = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor4_current = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor4_speed = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor4_encoder = 0;
            custom_msgs_readlkmotormulti_shared_msg.motor4_temperature = 0;

            publisher_multi_motor_->publish(custom_msgs_readlkmotormulti_shared_msg);
        } else {
            custom_msgs_readlkmotor_shared_msg.header.stamp = rclcpp::Clock().now();

            custom_msgs_readlkmotor_shared_msg.online = 0;
            custom_msgs_readlkmotor_shared_msg.enabled = 0;
            custom_msgs_readlkmotor_shared_msg.current = 0;
            custom_msgs_readlkmotor_shared_msg.speed = 0;
            custom_msgs_readlkmotor_shared_msg.encoder = 0;
            custom_msgs_readlkmotor_shared_msg.temperature = 0;

            publisher_single_motor_->publish(custom_msgs_readlkmotor_shared_msg);
        }
    }

    void LK_MOTOR::read() {
        shared_offset_ = pdoread_offset_;

        if (is_broadcast_mode_) {
            custom_msgs_readlkmotormulti_shared_msg.header.stamp = slave_device_->get_current_data_stamp();

            custom_msgs_readlkmotormulti_shared_msg.motor1_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor1_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor1_speed = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor1_encoder = read_uint16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor1_temperature = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);

            custom_msgs_readlkmotormulti_shared_msg.motor2_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor2_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor2_speed = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor2_encoder = read_uint16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor2_temperature = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);

            custom_msgs_readlkmotormulti_shared_msg.motor3_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor3_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor3_speed = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor3_encoder = read_uint16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor3_temperature = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);

            custom_msgs_readlkmotormulti_shared_msg.motor4_online = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor4_current = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor4_speed = read_int16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor4_encoder = read_uint16(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);
            custom_msgs_readlkmotormulti_shared_msg.motor4_temperature = read_uint8(
                slave_device_->get_slave_to_master_buf().data(),
                &shared_offset_);

            publisher_multi_motor_->publish(custom_msgs_readlkmotormulti_shared_msg);
        } else {
            custom_msgs_readlkmotor_shared_msg.header.stamp = slave_device_->get_current_data_stamp();

            const uint8_t state_byte = read_uint8(slave_device_->get_slave_to_master_buf().data(), &shared_offset_);
            custom_msgs_readlkmotor_shared_msg.online = state_byte & 0x01;
            custom_msgs_readlkmotor_shared_msg.enabled = state_byte >> 2 & 0x01;
            custom_msgs_readlkmotor_shared_msg.current = read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                    &shared_offset_);
            custom_msgs_readlkmotor_shared_msg.speed = read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                  &shared_offset_);
            custom_msgs_readlkmotor_shared_msg.encoder = read_uint16(slave_device_->get_slave_to_master_buf().data(),
                                                                     &shared_offset_);
            custom_msgs_readlkmotor_shared_msg.temperature = read_uint8(slave_device_->get_slave_to_master_buf().data(),
                                                                        &shared_offset_);

            publisher_single_motor_->publish(custom_msgs_readlkmotor_shared_msg);
        }
    }

    void LK_MOTOR::init_value() {
        int offset = pdowrite_offset_;

        if (is_broadcast_mode_) {
            write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);
            write_int16(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        } else {
            // simply write enable to 0
            // other args are not important
            write_uint8(0, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
    }

    void LK_MOTOR::on_command_openloop_control(custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->torque, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void LK_MOTOR::on_command_torque_control(custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->torque, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void LK_MOTOR::on_command_speed_with_torque_limit_control(
        custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->torque_limit, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int32(msg->speed, slave_device_->get_master_to_slave_buf().data(), &offset);
    }


    void LK_MOTOR::on_command_multi_round_pos_control(
        custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int32(msg->angle, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void LK_MOTOR::on_command_multi_round_pos_with_speed_limit_control(
        custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->speed_limit, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int32(msg->angle, slave_device_->get_master_to_slave_buf().data(), &offset);
    }


    void LK_MOTOR::on_command_single_round_pos_control(
        custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(msg->direction, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint32(msg->angle, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void LK_MOTOR::on_command_single_round_pos_with_speed_limit_control(
        custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint8(msg->enable, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint8(msg->direction, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->speed_limit, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint32(msg->angle, slave_device_->get_master_to_slave_buf().data(), &offset);
    }

    void LK_MOTOR::on_command_broadcast_current_control(
        custom_msgs::msg::WriteLkMotorBroadcastCurrentControl::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_int16(msg->motor1_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->motor2_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->motor3_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_int16(msg->motor4_cmd, slave_device_->get_master_to_slave_buf().data(), &offset);
    }
}
