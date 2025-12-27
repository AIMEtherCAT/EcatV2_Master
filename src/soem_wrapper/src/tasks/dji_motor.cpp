//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task {
    using namespace io::little_endian;
    using namespace utils::dynamic_data;
    using namespace dji_motor;

    custom_msgs::msg::ReadDJIMotor DJI_MOTOR::custom_msgs_readdjimotor_shared_msg;

    void DJI_MOTOR::init_sdo(uint8_t *buf, int *offset, const uint32_t /*sn*/, const uint8_t slave_id,
                             const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                {
                                                                    "connection_lost_write_action", "control_period",
                                                                    "can_packet_id", "motor1_can_id",
                                                                    "motor2_can_id", "motor3_can_id", "motor4_can_id",
                                                                    "can_inst"
                                                                });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        for (int i = 1; i <= 4; i++) {
            if (const std::string motor_arg_prefix = fmt::format("{}sdowrite_motor{}_", prefix, i);
                get_field_as<uint32_t>(
                    *get_dynamic_data(),
                    fmt::format("{}can_id", motor_arg_prefix)) > 0) {
                switch (std::get<uint8_t>(get_dynamic_data()->get(motor_arg_prefix + "control_type"))) {
                    case DJIMOTOR_CTRL_TYPE_CURRENT: {
                        auto [sdo_buf_ctrl_current, sdo_len_ctrl_current] = get_dynamic_data()->build_buf(
                            motor_arg_prefix, {"control_type"});
                        memcpy(buf + *offset, sdo_buf_ctrl_current, sdo_len_ctrl_current);
                        *offset += sdo_len_ctrl_current;
                        break;
                    }
                    case DJIMOTOR_CTRL_TYPE_SPEED: {
                        auto [sdo_buf_ctrl_speed, sdo_len_ctrl_speed] = get_dynamic_data()->build_buf(
                            motor_arg_prefix, {
                                "control_type", "speed_pid_kp", "speed_pid_ki",
                                "speed_pid_kd", "speed_pid_max_out",
                                "speed_pid_max_iout"
                            });
                        memcpy(buf + *offset, sdo_buf_ctrl_speed, sdo_len_ctrl_speed);
                        *offset += sdo_len_ctrl_speed;
                        break;
                    }
                    case DJIMOTOR_CTRL_TYPE_SINGLE_ROUND_POSITION: {
                        auto [sdo_buf_ctrl_pos, sdo_len_ctrl_pos] = get_dynamic_data()->build_buf(motor_arg_prefix, {
                                "control_type", "speed_pid_kp", "speed_pid_ki",
                                "speed_pid_kd", "speed_pid_max_out",
                                "speed_pid_max_iout",
                                "angle_pid_kp", "angle_pid_ki", "angle_pid_kd",
                                "angle_pid_max_out", "angle_pid_max_iout"
                            });
                        memcpy(buf + *offset, sdo_buf_ctrl_pos, sdo_len_ctrl_pos);
                        *offset += sdo_len_ctrl_pos;
                        break;
                    }
                    default: {
                    }
                }
            }
        }

        get_node()->create_and_insert_publisher<custom_msgs::msg::ReadDJIMotor>(prefix);
        get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteDJIMotor>(prefix, slave_id);
    }

    void DJI_MOTOR::publish_empty_message(const std::string &prefix) {
        custom_msgs_readdjimotor_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readdjimotor_shared_msg.motor1_online = 0;
        custom_msgs_readdjimotor_shared_msg.motor1_ecd = 0;
        custom_msgs_readdjimotor_shared_msg.motor1_rpm = 0;
        custom_msgs_readdjimotor_shared_msg.motor1_current = 0;
        custom_msgs_readdjimotor_shared_msg.motor1_temperature = 0;
        custom_msgs_readdjimotor_shared_msg.motor1_error_code = 0;

        custom_msgs_readdjimotor_shared_msg.motor2_online = 0;
        custom_msgs_readdjimotor_shared_msg.motor2_ecd = 0;
        custom_msgs_readdjimotor_shared_msg.motor2_rpm = 0;
        custom_msgs_readdjimotor_shared_msg.motor2_current = 0;
        custom_msgs_readdjimotor_shared_msg.motor2_temperature = 0;
        custom_msgs_readdjimotor_shared_msg.motor2_error_code = 0;

        custom_msgs_readdjimotor_shared_msg.motor3_online = 0;
        custom_msgs_readdjimotor_shared_msg.motor3_ecd = 0;
        custom_msgs_readdjimotor_shared_msg.motor3_rpm = 0;
        custom_msgs_readdjimotor_shared_msg.motor3_current = 0;
        custom_msgs_readdjimotor_shared_msg.motor3_temperature = 0;
        custom_msgs_readdjimotor_shared_msg.motor3_error_code = 0;

        custom_msgs_readdjimotor_shared_msg.motor4_online = 0;
        custom_msgs_readdjimotor_shared_msg.motor4_ecd = 0;
        custom_msgs_readdjimotor_shared_msg.motor4_rpm = 0;
        custom_msgs_readdjimotor_shared_msg.motor4_current = 0;
        custom_msgs_readdjimotor_shared_msg.motor4_temperature = 0;
        custom_msgs_readdjimotor_shared_msg.motor4_error_code = 0;

        EthercatNode::publish_msg<custom_msgs::msg::ReadDJIMotor>(prefix, custom_msgs_readdjimotor_shared_msg);
    }

    void DJI_MOTOR::init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) {
        write_uint8(0, buf, offset);
        write_int16(0, buf, offset);

        write_uint8(0, buf, offset);
        write_int16(0, buf, offset);

        write_uint8(0, buf, offset);
        write_int16(0, buf, offset);

        write_uint8(0, buf, offset);
        write_int16(0, buf, offset);
    }

    void DJI_MOTOR::read(const rclcpp::Time &stamp, const uint8_t *buf, int *offset, const std::string &prefix) {
        custom_msgs_readdjimotor_shared_msg.header.stamp = stamp;

        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor1_can_id", prefix), 0) > 0) {
            custom_msgs_readdjimotor_shared_msg.motor1_online = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor1_ecd = read_uint16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor1_rpm = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor1_current = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor1_temperature = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor1_error_code = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor2_can_id", prefix), 0) > 0) {
            custom_msgs_readdjimotor_shared_msg.motor2_online = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor2_ecd = read_uint16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor2_rpm = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor2_current = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor2_temperature = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor2_error_code = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor3_can_id", prefix), 0) > 0) {
            custom_msgs_readdjimotor_shared_msg.motor3_online = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor3_ecd = read_uint16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor3_rpm = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor3_current = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor3_temperature = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor3_error_code = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor4_can_id", prefix), 0) > 0) {
            custom_msgs_readdjimotor_shared_msg.motor4_online = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor4_ecd = read_uint16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor4_rpm = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor4_current = read_int16(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor4_temperature = read_uint8(buf, offset);
            custom_msgs_readdjimotor_shared_msg.motor4_error_code = read_uint8(buf, offset);
        }

        EthercatNode::publish_msg<custom_msgs::msg::ReadDJIMotor>(prefix, custom_msgs_readdjimotor_shared_msg);
    }

    void MsgDef<custom_msgs::msg::WriteDJIMotor>::write(const custom_msgs::msg::WriteDJIMotor::SharedPtr &msg,
                                                        uint8_t *buf, int *offset,
                                                        const std::string &prefix) {
        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor1_can_id", prefix)) > 0) {
            write_uint8(msg->motor1_enable, buf, offset);
            write_int16(msg->motor1_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor2_can_id", prefix)) > 0) {
            write_uint8(msg->motor2_enable, buf, offset);
            write_int16(msg->motor2_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor3_can_id", prefix)) > 0) {
            write_uint8(msg->motor3_enable, buf, offset);
            write_int16(msg->motor3_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(*get_dynamic_data(),
                                   fmt::format("{}sdowrite_motor4_can_id", prefix)) > 0) {
            write_uint8(msg->motor4_enable, buf, offset);
            write_int16(msg->motor4_cmd, buf, offset);
        }
    }
}
