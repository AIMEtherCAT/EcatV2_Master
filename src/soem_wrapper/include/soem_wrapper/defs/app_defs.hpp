//
// Created by hang on 25-6-27.
//

#ifndef APP_DEFS_HPP
#define APP_DEFS_HPP

#include "msg_defs.hpp"

extern std_msgs::msg::Float32 std_msgs_float32_shared_msg;
extern sensor_msgs::msg::Imu sensor_msgs_imu_shared_msg;
extern custom_msgs::msg::ReadDJIRC custom_msgs_readdjirc_shared_msg;
extern custom_msgs::msg::ReadDJICAN custom_msgs_readdjican_shared_msg;
extern custom_msgs::msg::ReadLkMotor custom_msgs_readlkmotor_shared_msg;
extern custom_msgs::msg::ReadMS5876BA30 custom_msgs_readms5876ba30_shared_msg;
extern custom_msgs::msg::ReadADC custom_msgs_readadc_shared_msg;
extern custom_msgs::msg::ReadCANPMU custom_msgs_readcanpmu_shared_msg;
extern custom_msgs::msg::ReadSBUSRC custom_msgs_readsbusrc_shared_msg;
extern custom_msgs::msg::ReadDmMotor custom_msgs_readdmmotor_shared_msg;

struct DJIRC
{
    static constexpr uint32_t type_id = DJIRC_APP_ID;
    static constexpr auto type_enum = "DJIRC";
    static controller_t rc_ctrl_raw;

    static void
    init_sdo(uint8_t* /*buf*/, int* /*offset*/, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        node->create_and_insert_publisher<custom_msgs::msg::ReadDJIRC>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readdjirc_shared_msg.header.stamp = rclcpp::Clock().now();

        if (buf[*offset + 18])
        {
            // channel
            custom_msgs_readdjirc_shared_msg.right_x = get_percentage(
                (buf[*offset + 0] | buf[*offset + 1] << 8) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.right_y = get_percentage(
                (buf[*offset + 1] >> 3 | buf[*offset + 2] << 5) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.left_x = get_percentage(
                (buf[*offset + 2] >> 6 | buf[*offset + 3] << 2 | buf[*offset + 4] << 10) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.left_y = get_percentage(
                (buf[*offset + 4] >> 1 | buf[*offset + 5] << 7) & 0x07ff);
            custom_msgs_readdjirc_shared_msg.dial = get_percentage(
                (buf[*offset + 16] | buf[*offset + 17] << 8) & 0x07ff);

            // switcher
            custom_msgs_readdjirc_shared_msg.right_switch = (buf[*offset + 5] >> 4) & 0x03;
            custom_msgs_readdjirc_shared_msg.left_switch = (buf[*offset + 5] >> 6) & 0x03;

            // mouse
            custom_msgs_readdjirc_shared_msg.mouse_x = static_cast<int16_t>(buf[*offset + 6] | buf[*offset + 7] << 8);
            custom_msgs_readdjirc_shared_msg.mouse_y = static_cast<int16_t>(buf[*offset + 8] | buf[*offset + 9] << 8);
            custom_msgs_readdjirc_shared_msg.mouse_left_clicked = buf[*offset + 12];
            custom_msgs_readdjirc_shared_msg.mouse_right_clicked = buf[*offset + 13];

            // keyboard
            const uint16_t key_value = static_cast<uint16_t>(buf[*offset + 14] | buf[*offset + 15] << 8);
            custom_msgs_readdjirc_shared_msg.w = (key_value >> 0) & 0x01;
            custom_msgs_readdjirc_shared_msg.s = (key_value >> 1) & 0x01;
            custom_msgs_readdjirc_shared_msg.a = (key_value >> 2) & 0x01;
            custom_msgs_readdjirc_shared_msg.d = (key_value >> 3) & 0x01;
            custom_msgs_readdjirc_shared_msg.shift = (key_value >> 4) & 0x01;
            custom_msgs_readdjirc_shared_msg.ctrl = (key_value >> 5) & 0x01;
            custom_msgs_readdjirc_shared_msg.q = (key_value >> 6) & 0x01;
            custom_msgs_readdjirc_shared_msg.e = (key_value >> 7) & 0x01;
            custom_msgs_readdjirc_shared_msg.r = (key_value >> 8) & 0x01;
            custom_msgs_readdjirc_shared_msg.f = (key_value >> 9) & 0x01;
            custom_msgs_readdjirc_shared_msg.g = (key_value >> 10) & 0x01;
            custom_msgs_readdjirc_shared_msg.z = (key_value >> 11) & 0x01;
            custom_msgs_readdjirc_shared_msg.x = (key_value >> 12) & 0x01;
            custom_msgs_readdjirc_shared_msg.c = (key_value >> 13) & 0x01;
            custom_msgs_readdjirc_shared_msg.v = (key_value >> 14) & 0x01;
            custom_msgs_readdjirc_shared_msg.b = (key_value >> 15) & 0x01;

            custom_msgs_readdjirc_shared_msg.online = 1;
        }
        else
        {
            custom_msgs_readdjirc_shared_msg.online = 0;
        }

        *offset += 19;
        EthercatNode::publish_msg<custom_msgs::msg::ReadDJIRC>(prefix, custom_msgs_readdjirc_shared_msg);
    }
};

struct HIPNUC_IMU
{
    static constexpr uint32_t type_id = HIPNUC_IMU_CAN_APP_ID;
    static constexpr auto type_enum = "HIPNUC_IMU";

    static void
    init_sdo(uint8_t* /*buf*/, int* /*offset*/, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        node->create_and_insert_publisher<sensor_msgs::msg::Imu>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        sensor_msgs_imu_shared_msg.header.stamp = rclcpp::Clock().now();
        sensor_msgs_imu_shared_msg.header.frame_id = "imu_link";

        // hipnuc hi92 protocol
        sensor_msgs_imu_shared_msg.orientation.w = 0.0001 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.orientation.x = 0.0001 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.orientation.y = 0.0001 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.orientation.z = 0.0001 * read_int16(buf, offset);

        sensor_msgs_imu_shared_msg.linear_acceleration.x = 0.0048828 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.linear_acceleration.y = 0.0048828 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.linear_acceleration.z = 0.0048828 * read_int16(buf, offset);

        sensor_msgs_imu_shared_msg.angular_velocity.x = 0.001 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.angular_velocity.y = 0.001 * read_int16(buf, offset);
        sensor_msgs_imu_shared_msg.angular_velocity.z = 0.001 * read_int16(buf, offset);

        EthercatNode::publish_msg<sensor_msgs::msg::Imu>(prefix, sensor_msgs_imu_shared_msg);
    }
};

struct DJI_MOTOR
{
    static constexpr uint32_t type_id = DJICAN_APP_ID;
    static constexpr auto type_enum = "DJI_MOTOR";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {
                                      "control_period", "can_packet_id", "motor1_can_id",
                                      "motor2_can_id", "motor3_can_id", "motor4_can_id", "can_inst"
                                  }),
               23);
        *offset += 23;

        for (int i = 1; i <= 4; i++)
        {
            if (const std::string motor_arg_prefix = fmt::format("{}sdowrite_motor{}_", prefix, i);
                get_field_as<uint32_t>(fmt::format("{}can_id", motor_arg_prefix)) > 0)
            {
                switch (std::get<uint8_t>(sdo_data.get(motor_arg_prefix + "control_type")))
                {
                case DJIMOTOR_CTRL_TYPE_CURRENT:
                    {
                        memcpy(buf + *offset,
                               sdo_data.build_buf(motor_arg_prefix, {"control_type"}), 1);
                        *offset += 1;
                        break;
                    }
                case DJIMOTOR_CTRL_TYPE_SPEED:
                    {
                        memcpy(
                            buf + *offset,
                            sdo_data.build_buf(motor_arg_prefix, {
                                                   "control_type", "speed_pid_kp", "speed_pid_ki",
                                                   "speed_pid_kd", "speed_pid_max_out", "speed_pid_max_iout"
                                               }),
                            21);
                        *offset += 21;
                        break;
                    }
                case DJIMOTOR_CTRL_TYPE_SINGLE_ROUND_POSITION:
                    {
                        memcpy(
                            buf + *offset,
                            sdo_data.build_buf(motor_arg_prefix, {
                                                   "control_type", "speed_pid_kp", "speed_pid_ki",
                                                   "speed_pid_kd", "speed_pid_max_out", "speed_pid_max_iout",
                                                   "angle_pid_kp", "angle_pid_ki", "angle_pid_kd",
                                                   "angle_pid_max_out", "angle_pid_max_iout"
                                               }),
                            41);
                        *offset += 41;
                        break;
                    }
                default:
                    {
                    }
                }
            }
        }

        node->create_and_insert_publisher<custom_msgs::msg::ReadDJICAN>(prefix);
        node->create_and_insert_subscriber<custom_msgs::msg::WriteDJICAN>(prefix, slave_id);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readdjican_shared_msg.header.stamp = rclcpp::Clock().now();

        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor1_can_id", prefix), 0) > 0)
        {
            custom_msgs_readdjican_shared_msg.motor1_ecd = read_uint16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor1_rpm = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor1_current = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor1_temperature = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor2_can_id", prefix), 0) > 0)
        {
            custom_msgs_readdjican_shared_msg.motor2_ecd = read_uint16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor2_rpm = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor2_current = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor2_temperature = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor3_can_id", prefix), 0) > 0)
        {
            custom_msgs_readdjican_shared_msg.motor3_ecd = read_uint16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor3_rpm = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor3_current = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor3_temperature = read_uint8(buf, offset);
        }

        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor4_can_id", prefix), 0) > 0)
        {
            custom_msgs_readdjican_shared_msg.motor4_ecd = read_uint16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor4_rpm = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor4_current = read_int16(buf, offset);
            custom_msgs_readdjican_shared_msg.motor4_temperature = read_uint8(buf, offset);
        }

        EthercatNode::publish_msg<custom_msgs::msg::ReadDJICAN>(prefix, custom_msgs_readdjican_shared_msg);
    }
};

struct DSHOT
{
    static constexpr uint32_t type_id = DSHOT_APP_ID;
    static constexpr auto type_enum = "DSHOT";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"dshot_id"}),
               1);
        *offset += 1;
        node->create_and_insert_subscriber<custom_msgs::msg::WriteDSHOT>(prefix, slave_id);
    }
};

struct VANILLA_PWM
{
    static constexpr uint32_t type_id = VANILLA_PWM_APP_ID;
    static constexpr auto type_enum = "VANILLA_PWM";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"tim_id", "pwm_period", "init_value"}),
               5);
        *offset += 5;
        node->create_and_insert_subscriber<custom_msgs::msg::WriteVanillaPWM>(prefix, slave_id);
    }

    static void
    init_value(uint8_t* buf, int* offset, const std::string& prefix)
    {
        const uint16_t init_value = get_field_as<uint16_t>(fmt::format("{}sdowrite_init_value", prefix));
        for (int i = 1; i <= 4; i++)
        {
            RCLCPP_INFO(cfg_logger, "prefix=%s will write init value=%d at m2s buf idx=%d", prefix.c_str(), init_value,
                        *offset);
            write_uint16(init_value, buf, offset);
        }
    }
};

struct EXTERNAL_PWM
{
    static constexpr uint32_t type_id = EXTERNAL_PWM_APP_ID;
    static constexpr auto type_enum = "EXTERNAL_PWM";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"uart_id", "pwm_period", "channel_num", "init_value"}),
               6);
        *offset += 6;
        node->create_and_insert_subscriber<custom_msgs::msg::WriteExternalPWM>(prefix, slave_id);
    }

    static void
    init_value(uint8_t* buf, int* offset, const std::string& prefix)
    {
        const uint16_t init_value = get_field_as<uint16_t>(fmt::format("{}sdowrite_init_value", prefix));
        const uint8_t channel_num = get_field_as<uint8_t>(fmt::format("{}sdowrite_channel_num", prefix));

        for (int i = 1; i <= channel_num; i++)
        {
            RCLCPP_INFO(cfg_logger, "prefix=%s will write init value=%d at m2s buf idx=%d", prefix.c_str(), init_value,
                        *offset);
            write_uint16(init_value, buf, offset);
        }
    }
};

struct MS5876_30BA
{
    static constexpr uint32_t type_id = MS5876_30BA_APP_ID;
    static constexpr auto type_enum = "MS5876_30BA";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"i2c_id", "osr_id"}),
               2);
        *offset += 2;
        node->create_and_insert_publisher<custom_msgs::msg::ReadMS5876BA30>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readms5876ba30_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readms5876ba30_shared_msg.temperature = read_int32(buf, offset) / 100.;
        custom_msgs_readms5876ba30_shared_msg.pressure = read_int32(buf, offset) / 10.;

        EthercatNode::publish_msg<custom_msgs::msg::ReadMS5876BA30>(prefix, custom_msgs_readms5876ba30_shared_msg);
    }
};

struct LK_MOTOR
{
    static constexpr uint32_t type_id = LK_APP_ID;
    static constexpr auto type_enum = "LK_MOTOR";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"control_period", "can_packet_id", "can_inst", "control_type"}),
               8);
        *offset += 8;

        switch (get_field_as<uint8_t>(fmt::format("{}sdowrite_control_type", prefix)))
        {
        case LK_CTRL_TYPE_OPENLOOP_CURRENT:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorOpenloopControl>(prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_TORQUE:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorTorqueControl>(prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_SPEED_WITH_TORQUE_LIMIT:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
                    prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_MULTI_ROUND_POSITION:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>(
                    prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_MULTI_ROUND_POSITION_WITH_SPEED_LIMIT:
            {
                node->create_and_insert_subscriber<
                    custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_SINGLE_ROUND_POSITION:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>(
                    prefix, slave_id);
                break;
            }

        case LK_CTRL_TYPE_SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT:
            {
                node->create_and_insert_subscriber<
                    custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }
        default:
            {
            }
        }

        node->create_and_insert_publisher<custom_msgs::msg::ReadLkMotor>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readlkmotor_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readlkmotor_shared_msg.current = read_int16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.rpm = read_int16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.encoder = read_uint16(buf, offset);
        custom_msgs_readlkmotor_shared_msg.temperature = read_uint8(buf, offset);

        EthercatNode::publish_msg<custom_msgs::msg::ReadLkMotor>(prefix, custom_msgs_readlkmotor_shared_msg);
    }
};

struct ADC
{
    static constexpr uint32_t type_id = ADC_APP_ID;
    static constexpr auto type_enum = "ADC";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"channel1_coefficient_per_volt", "channel2_coefficient_per_volt"}),
               8);
        *offset += 8;
        node->create_and_insert_publisher<custom_msgs::msg::ReadADC>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readadc_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readadc_shared_msg.channel1 = read_float(buf, offset);
        custom_msgs_readadc_shared_msg.channel2 = read_float(buf, offset);

        EthercatNode::publish_msg<custom_msgs::msg::ReadADC>(prefix, custom_msgs_readadc_shared_msg);
    }
};

struct CAN_PMU
{
    static constexpr uint32_t type_id = CAN_PMU_APP_ID;
    static constexpr auto type_enum = "CAN_PMU";

    static void
    init_sdo(uint8_t* /*buf*/, int* /*offset*/, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        node->create_and_insert_publisher<custom_msgs::msg::ReadCANPMU>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readcanpmu_shared_msg.header.stamp = rclcpp::Clock().now();

        custom_msgs_readcanpmu_shared_msg.temperature = read_float16(buf, offset) - 273.15f;
        custom_msgs_readcanpmu_shared_msg.voltage = read_float16(buf, offset);
        custom_msgs_readcanpmu_shared_msg.current = read_float16(buf, offset);

        EthercatNode::publish_msg<custom_msgs::msg::ReadCANPMU>(prefix, custom_msgs_readcanpmu_shared_msg);
    }
};

struct SBUSRC
{
    static constexpr uint32_t type_id = SBUS_RC_APP_ID;
    static constexpr auto type_enum = "SBUSRS";

    static void
    init_sdo(uint8_t* /*buf*/, int* /*offset*/, const uint32_t& /*sn*/, const uint8_t /*slave_id*/,
             const std::string& prefix)
    {
        node->create_and_insert_publisher<custom_msgs::msg::ReadSBUSRC>(prefix);
        custom_msgs_readsbusrc_shared_msg.channels.resize(16);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
        custom_msgs_readsbusrc_shared_msg.header.stamp = rclcpp::Clock().now();

        if (buf[*offset + 23])
        {
            custom_msgs_readsbusrc_shared_msg.channels[0] = ((buf[*offset + 0] | buf[*offset + 1] << 8) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[1] = ((buf[*offset + 1] >> 3 | buf[*offset + 2] << 5) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[2] = ((buf[*offset + 2] >> 6 | buf[*offset + 3] << 2 | buf[*
                offset + 4] << 10) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[3] = ((buf[*offset + 4] >> 1 | buf[*offset + 5] << 7) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[4] = ((buf[*offset + 5] >> 4 | buf[*offset + 6] << 4) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[5] = ((buf[*offset + 6] >> 7 | buf[*offset + 7] << 1 | buf[*
                offset + 8] << 9) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[6] = ((buf[*offset + 8] >> 2 | buf[*offset + 9] << 6) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[7] = ((buf[*offset + 9] >> 5 | buf[*offset + 10] << 3) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[8] = ((buf[*offset + 11] | buf[*offset + 12] << 8) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[9] = ((buf[*offset + 12] >> 3 | buf[*offset + 13] << 5) &
                0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[10] = ((buf[*offset + 13] >> 6 | buf[*offset + 14] << 2 | buf[*
                offset + 15] << 10) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[11] = ((buf[*offset + 15] >> 1 | buf[*offset + 16] << 7) &
                0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[12] = ((buf[*offset + 16] >> 4 | buf[*offset + 17] << 4) &
                0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[13] = ((buf[*offset + 17] >> 7 | buf[*offset + 18] << 1 | buf[*
                offset + 19] << 9) & 0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[14] = ((buf[*offset + 19] >> 2 | buf[*offset + 20] << 6) &
                0x07FF);
            custom_msgs_readsbusrc_shared_msg.channels[15] = ((buf[*offset + 20] >> 5 | buf[*offset + 21] << 3) &
                0x07FF);

            custom_msgs_readsbusrc_shared_msg.ch17 = buf[*offset + 22] & 0x01 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.ch18 = buf[*offset + 22] & 0x02 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.fail_safe = buf[*offset + 22] & 0x04 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.frame_lost = buf[*offset + 22] & 0x08 ? 1 : 0;

            custom_msgs_readsbusrc_shared_msg.online = 1;
        }
        else
        {
            custom_msgs_readsbusrc_shared_msg.online = 0;
        }

        *offset += 24;
        EthercatNode::publish_msg<custom_msgs::msg::ReadSBUSRC>(prefix, custom_msgs_readsbusrc_shared_msg);
    }
};

struct DM_MOTOR
{
    static constexpr uint32_t type_id = DM_MOTOR_APP_ID;
    static constexpr auto type_enum = "DM_MOTOR";

    static void
    init_sdo(uint8_t* buf, int* offset, const uint32_t& /*sn*/, const uint8_t slave_id,
             const std::string& prefix)
    {
        memcpy(buf + *offset,
               sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
                                  {"control_period", "can_id", "master_id", "can_inst", "control_type"}),
               8);
        *offset += 8;

        switch (get_field_as<uint8_t>(fmt::format("{}sdowrite_control_type", prefix)))
        {
        case DM_CTRL_TYPE_MIT:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorMITControl>(prefix, slave_id);
                break;
            }

        case DM_CTRL_TYPE_POSITION_WITH_SPEED_LIMIT:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
                    prefix, slave_id);
                break;
            }

        case DM_CTRL_TYPE_SPEED:
            {
                node->create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorSpeedControl>(
                    prefix, slave_id);
                break;
            }

        default:
            {
            }
        }

        node->create_and_insert_publisher<custom_msgs::msg::ReadDmMotor>(prefix);
    }

    static void
    read(const uint8_t* buf, int* offset, const std::string& prefix)
    {
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

        if (buf[*offset + 8] == 0)
        {
            custom_msgs_readdmmotor_shared_msg.online = 0;
            custom_msgs_readdmmotor_shared_msg.ecd = 0;
            custom_msgs_readdmmotor_shared_msg.velocity = 0;
            custom_msgs_readdmmotor_shared_msg.torque = 0;
            custom_msgs_readdmmotor_shared_msg.mos_temperature = 0;
            custom_msgs_readdmmotor_shared_msg.rotor_temperature = 0;
        }
        else
        {
            custom_msgs_readdmmotor_shared_msg.online = 1;

            switch (buf[*offset + 0] >> 4)
            {
            case 0x0:
                {
                    custom_msgs_readdmmotor_shared_msg.disabled = 1;
                    break;
                }
            case 0x1:
                {
                    custom_msgs_readdmmotor_shared_msg.enabled = 1;
                    break;
                }
            case 0x8:
                {
                    custom_msgs_readdmmotor_shared_msg.overvoltage = 1;
                    break;
                }
            case 0x9:
                {
                    custom_msgs_readdmmotor_shared_msg.undervoltage = 1;
                    break;
                }
            case 0xA:
                {
                    custom_msgs_readdmmotor_shared_msg.overcurrent = 1;
                    break;
                }
            case 0xB:
                {
                    custom_msgs_readdmmotor_shared_msg.mos_overtemperature = 1;
                    break;
                }
            case 0xC:
                {
                    custom_msgs_readdmmotor_shared_msg.rotor_overtemperature = 1;
                    break;
                }
            case 0xD:
                {
                    custom_msgs_readdmmotor_shared_msg.communication_lost = 1;
                    break;
                }
            case 0xE:
                {
                    custom_msgs_readdmmotor_shared_msg.overload = 1;
                    break;
                }
            default:
                {
                }
            }

            custom_msgs_readdmmotor_shared_msg.ecd = ((buf[*offset + 1] << 8) | buf[*offset + 2]) % 16384;
            custom_msgs_readdmmotor_shared_msg.velocity = uint_to_float(
                ((buf[*offset + 3] << 4) | (buf[*offset + 4] >> 4)),
                -get_field_as<float>(fmt::format("{}sdowrite_vmax", prefix)),
                get_field_as<float>(fmt::format("{}sdowrite_vmax", prefix)),
                12);
            custom_msgs_readdmmotor_shared_msg.torque = uint_to_float(
                ((buf[*offset + 4] & 0xF) << 8) | buf[*offset + 5],
                -get_field_as<float>(fmt::format("{}sdowrite_tmax", prefix)),
                get_field_as<float>(fmt::format("{}sdowrite_tmax", prefix)),
                12);
            custom_msgs_readdmmotor_shared_msg.mos_temperature = buf[*offset + 6];
            custom_msgs_readdmmotor_shared_msg.rotor_temperature = buf[*offset + 7];
        }

        *offset += 9;
        EthercatNode::publish_msg<custom_msgs::msg::ReadDmMotor>(prefix, custom_msgs_readdmmotor_shared_msg);
    }

    static void
    init_value(uint8_t* buf, int* offset, const std::string& prefix)
    {
        write_uint8(0, buf, offset);
        if (get_field_as<uint8_t>(fmt::format("{}sdowrite_control_type", prefix)) <= 2)
        {
            write_float(0, buf, offset);
            write_float(0, buf, offset);
        } else
        {
            write_float(0, buf, offset);
        }
    }
};

#endif //APP_DEFS_HPP
