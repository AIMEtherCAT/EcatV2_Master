//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::dynamic_data;
    using namespace lk_motor;

    custom_msgs::msg::ReadLkMotor LK_MOTOR::custom_msgs_readlkmotor_shared_msg;

    void LK_MOTOR::init_sdo(uint8_t *buf, int *offset, const uint32_t /*sn*/, const uint8_t slave_id,
                            const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                     {
                                                         "connection_lost_write_action", "control_period",
                                                         "can_packet_id", "can_inst", "control_type"
                                                     });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        switch (get_field_as<uint8_t>(*get_dynamic_data(),
            fmt::format("{}sdowrite_control_type", prefix))) {
            case LK_CTRL_TYPE_OPENLOOP_CURRENT: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorOpenloopControl>(prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_TORQUE: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorTorqueControl>(prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_SPEED_WITH_TORQUE_LIMIT: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
                    prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_MULTI_ROUND_POSITION: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>(
                    prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                get_node()->create_and_insert_subscriber<
                    custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_SINGLE_ROUND_POSITION: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>(
                    prefix, slave_id);
                break;
            }

            case LK_CTRL_TYPE_SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                get_node()->create_and_insert_subscriber<
                    custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }
            default: {
            }
        }

        get_node()->create_and_insert_publisher<custom_msgs::msg::ReadLkMotor>(prefix);
    }

    void LK_MOTOR::publish_empty_message(const std::string &prefix) {
        custom_msgs_readlkmotor_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readlkmotor_shared_msg.online = 0;
        custom_msgs_readlkmotor_shared_msg.enabled = 0;
        custom_msgs_readlkmotor_shared_msg.current = 0;
        custom_msgs_readlkmotor_shared_msg.speed = 0;
        custom_msgs_readlkmotor_shared_msg.encoder = 0;
        custom_msgs_readlkmotor_shared_msg.temperature = 0;

        EthercatNode::publish_msg<custom_msgs::msg::ReadLkMotor>(prefix, custom_msgs_readlkmotor_shared_msg);
    }

    void LK_MOTOR::read(const uint8_t *buf, int *offset, const std::string &prefix) {
        custom_msgs_readlkmotor_shared_msg.header.stamp = rclcpp::Clock().now();

        const uint8_t state_byte = read_uint8(buf, offset);
        custom_msgs_readlkmotor_shared_msg.online = state_byte & 0x01;
        custom_msgs_readlkmotor_shared_msg.enabled = state_byte >> 2 & 0x01;
        custom_msgs_readlkmotor_shared_msg.current = read_int16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.speed = read_int16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.encoder = read_uint16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.temperature = read_uint8(buf, offset);

        EthercatNode::publish_msg<custom_msgs::msg::ReadLkMotor>(prefix, custom_msgs_readlkmotor_shared_msg);
    }

    void LK_MOTOR::init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) {
        // simply write enable to 0
        // other args are not important
        write_uint8(0, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteLkMotorOpenloopControl>::write(
        const custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteLkMotorTorqueControl>::write(
        const custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>::write(
        const custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr &msg, uint8_t *buf,
        int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque_limit, buf, offset);
        write_int32(msg->speed, buf, offset);
    }


    void MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>::write(
        const custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_int32(msg->angle, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>::write(
        const custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr &msg, uint8_t *buf,
        int *offset, const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->speed_limit, buf, offset);
        write_int32(msg->angle, buf, offset);
    }


    void MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>::write(
        const custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_uint8(msg->direction, buf, offset);
        write_uint32(msg->angle, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>::write(
        const custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr &msg,
        uint8_t *buf,
        int *offset, const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_uint8(msg->direction, buf, offset);
        write_int16(msg->speed_limit, buf, offset);
        write_uint32(msg->angle, buf, offset);
    }
}
