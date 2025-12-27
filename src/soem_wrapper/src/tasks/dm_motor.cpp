//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task {
    using namespace aim::io::little_endian;
    using namespace utils::dynamic_data;
    using namespace dm_motor;

    custom_msgs::msg::ReadDmMotor DM_MOTOR::custom_msgs_readdmmotor_shared_msg;

    void DM_MOTOR::init_sdo(uint8_t *buf, int *offset, const uint32_t /*sn*/, const uint8_t slave_id,
                            const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                {
                                                                    "connection_lost_write_action", "control_period",
                                                                    "can_id",
                                                                    "master_id", "can_inst", "control_type"
                                                                });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        switch (get_field_as<uint8_t>(*get_dynamic_data(),
                                      fmt::format("{}sdowrite_control_type", prefix))) {
            case DM_CTRL_TYPE_MIT: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorMITControl>(prefix, slave_id);
                break;
            }

            case DM_CTRL_TYPE_POSITION_WITH_SPEED_LIMIT: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }

            case DM_CTRL_TYPE_SPEED: {
                get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorSpeedControl>(
                    prefix, slave_id);
                break;
            }

            default: {
            }
        }

        get_node()->create_and_insert_publisher<custom_msgs::msg::ReadDmMotor>(prefix);
    }

    void DM_MOTOR::publish_empty_message(const std::string &prefix) {
        custom_msgs_readdmmotor_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readdmmotor_shared_msg.disabled = 0;
        custom_msgs_readdmmotor_shared_msg.enabled = 0;
        custom_msgs_readdmmotor_shared_msg.overvoltage = 0;
        custom_msgs_readdmmotor_shared_msg.undervoltage = 0;
        custom_msgs_readdmmotor_shared_msg.overcurrent = 0;
        custom_msgs_readdmmotor_shared_msg.mos_overtemperature = 0;
        custom_msgs_readdmmotor_shared_msg.rotor_overtemperature = 0;
        custom_msgs_readdmmotor_shared_msg.communication_lost = 0;
        custom_msgs_readdmmotor_shared_msg.overload = 0;

        custom_msgs_readdmmotor_shared_msg.online = 0;
        custom_msgs_readdmmotor_shared_msg.ecd = 0;
        custom_msgs_readdmmotor_shared_msg.velocity = 0;
        custom_msgs_readdmmotor_shared_msg.torque = 0;
        custom_msgs_readdmmotor_shared_msg.mos_temperature = 0;
        custom_msgs_readdmmotor_shared_msg.rotor_temperature = 0;

        EthercatNode::publish_msg<custom_msgs::msg::ReadDmMotor>(prefix, custom_msgs_readdmmotor_shared_msg);
    }

    static int float_to_uint(const float x_float, const float x_min, const float x_max, const int bits) { // NOLINT
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<int>((x_float - offset) * static_cast<float>((1 << bits) - 1) / span);
    }

    static float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) { // NOLINT
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
    }

    void DM_MOTOR::read(const rclcpp::Time &stamp, const uint8_t *buf, int *offset, const std::string &prefix) { // NOLINT
        custom_msgs_readdmmotor_shared_msg.header.stamp = stamp;

        custom_msgs_readdmmotor_shared_msg.disabled = 0;
        custom_msgs_readdmmotor_shared_msg.enabled = 0;
        custom_msgs_readdmmotor_shared_msg.overvoltage = 0;
        custom_msgs_readdmmotor_shared_msg.undervoltage = 0;
        custom_msgs_readdmmotor_shared_msg.overcurrent = 0;
        custom_msgs_readdmmotor_shared_msg.mos_overtemperature = 0;
        custom_msgs_readdmmotor_shared_msg.rotor_overtemperature = 0;
        custom_msgs_readdmmotor_shared_msg.communication_lost = 0;
        custom_msgs_readdmmotor_shared_msg.overload = 0;

        if (buf[*offset + 8] == 0) {
            custom_msgs_readdmmotor_shared_msg.online = 0;
            custom_msgs_readdmmotor_shared_msg.ecd = 0;
            custom_msgs_readdmmotor_shared_msg.velocity = 0;
            custom_msgs_readdmmotor_shared_msg.torque = 0;
            custom_msgs_readdmmotor_shared_msg.mos_temperature = 0;
            custom_msgs_readdmmotor_shared_msg.rotor_temperature = 0;
        } else {
            custom_msgs_readdmmotor_shared_msg.online = 1;

            switch (buf[*offset + 0] >> 4) {
                case 0x0: {
                    custom_msgs_readdmmotor_shared_msg.disabled = 1;
                    break;
                }
                case 0x1: {
                    custom_msgs_readdmmotor_shared_msg.enabled = 1;
                    break;
                }
                case 0x8: {
                    custom_msgs_readdmmotor_shared_msg.overvoltage = 1;
                    break;
                }
                case 0x9: {
                    custom_msgs_readdmmotor_shared_msg.undervoltage = 1;
                    break;
                }
                case 0xA: {
                    custom_msgs_readdmmotor_shared_msg.overcurrent = 1;
                    break;
                }
                case 0xB: {
                    custom_msgs_readdmmotor_shared_msg.mos_overtemperature = 1;
                    break;
                }
                case 0xC: {
                    custom_msgs_readdmmotor_shared_msg.rotor_overtemperature = 1;
                    break;
                }
                case 0xD: {
                    custom_msgs_readdmmotor_shared_msg.communication_lost = 1;
                    break;
                }
                case 0xE: {
                    custom_msgs_readdmmotor_shared_msg.overload = 1;
                    break;
                }
                default: {
                }
            }

            custom_msgs_readdmmotor_shared_msg.ecd = buf[*offset + 1] << 8 | buf[*offset + 2];
            custom_msgs_readdmmotor_shared_msg.velocity = uint_to_float(
                buf[*offset + 3] << 4 | buf[*offset + 4] >> 4,
                -get_field_as<float>(*get_dynamic_data(), fmt::format("{}conf_vmax", prefix)),
                get_field_as<float>(*get_dynamic_data(), fmt::format("{}conf_vmax", prefix)),
                12);
            custom_msgs_readdmmotor_shared_msg.torque = uint_to_float(
                (buf[*offset + 4] & 0xF) << 8 | buf[*offset + 5],
                -get_field_as<float>(*get_dynamic_data(), fmt::format("{}conf_tmax", prefix)),
                get_field_as<float>(*get_dynamic_data(), fmt::format("{}conf_tmax", prefix)),
                12);
            custom_msgs_readdmmotor_shared_msg.mos_temperature = buf[*offset + 6];
            custom_msgs_readdmmotor_shared_msg.rotor_temperature = buf[*offset + 7];
        }

        EthercatNode::publish_msg<custom_msgs::msg::ReadDmMotor>(prefix, custom_msgs_readdmmotor_shared_msg);
    }

    void DM_MOTOR::init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) {
        // simply write enable to 0
        // other args are not important
        write_uint8(0, buf, offset);
    }

    void MsgDef<custom_msgs::msg::WriteDmMotorMITControl>::write(
        const custom_msgs::msg::WriteDmMotorMITControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string &prefix) {
        write_uint8(msg->enable, buf, offset);
        static uint8_t WriteDmMotorMITControl_data[8] = {};

        const uint16_t writeDmMotorMITControl_pos = float_to_uint(msg->p_des,
                                                                  -get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_pmax", prefix)),
                                                                  get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_pmax", prefix)),
                                                                  16);
        const uint16_t writeDmMotorMITControl_vel = float_to_uint(msg->v_des,
                                                                  -get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_vmax", prefix)),
                                                                  get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_vmax", prefix)),
                                                                  12);
        const uint16_t writeDmMotorMITControl_tor = float_to_uint(msg->torque,
                                                                  -get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_tmax", prefix)),
                                                                  get_field_as<float>(
                                                                      *get_dynamic_data(),
                                                                      fmt::format("{}sdowrite_tmax", prefix)),
                                                                  12);
        const uint16_t writeDmMotorMITControl_kp = float_to_uint(msg->kp, 0.0, 500.0, 12);
        const uint16_t writeDmMotorMITControl_kd = float_to_uint(msg->kd, 0.0, 5.0, 12);

        WriteDmMotorMITControl_data[0] = writeDmMotorMITControl_pos >> 8;
        WriteDmMotorMITControl_data[1] = writeDmMotorMITControl_pos;
        WriteDmMotorMITControl_data[2] = writeDmMotorMITControl_vel >> 4;
        WriteDmMotorMITControl_data[3] = (writeDmMotorMITControl_vel & 0xF) << 4 | writeDmMotorMITControl_kp >> 8;
        WriteDmMotorMITControl_data[4] = writeDmMotorMITControl_kp;
        WriteDmMotorMITControl_data[5] = writeDmMotorMITControl_kd >> 4;
        WriteDmMotorMITControl_data[6] = (writeDmMotorMITControl_kd & 0xF) << 4 | writeDmMotorMITControl_tor >> 8;
        WriteDmMotorMITControl_data[7] = writeDmMotorMITControl_tor;

        memcpy(buf + *offset, WriteDmMotorMITControl_data, 8);
        *offset += 8;
    }


    void MsgDef<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::write(
        const custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit::SharedPtr &msg, uint8_t *buf,
        int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_float(msg->position, buf, offset);
        write_float(msg->speed, buf, offset);
    }


    void MsgDef<custom_msgs::msg::WriteDmMotorSpeedControl>::write(
        const custom_msgs::msg::WriteDmMotorSpeedControl::SharedPtr &msg, uint8_t *buf, int *offset,
        const std::string & /*prefix*/) {
        write_uint8(msg->enable, buf, offset);
        write_float(msg->speed, buf, offset);
    }
}
