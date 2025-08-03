//
// Created by hang on 25-6-25.
//

#ifndef MSG_DEFS_HPP
#define MSG_DEFS_HPP

#include "../ethercat_node.hpp"

template <>
struct MsgDef<custom_msgs::msg::WriteDSHOT>
{
    static constexpr auto type_enum = "WriteDSHOT";

    static void
    write(const custom_msgs::msg::WriteDSHOT::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint16(msg->channel1, buf, offset);
        write_uint16(msg->channel2, buf, offset);
        write_uint16(msg->channel3, buf, offset);
        write_uint16(msg->channel4, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteVanillaPWM>
{
    static constexpr auto type_enum = "WriteVanillaPWM";

    static void
    write(const custom_msgs::msg::WriteVanillaPWM::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint16(msg->channel1, buf, offset);
        write_uint16(msg->channel2, buf, offset);
        write_uint16(msg->channel3, buf, offset);
        write_uint16(msg->channel4, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorOpenloopControl>
{
    static constexpr auto type_enum = "WriteVanillaPWM";

    static void
    write(const custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorTorqueControl>
{
    static constexpr auto type_enum = "WriteLkMotorTorqueControl";

    static void
    write(const custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>
{
    static constexpr auto type_enum = "WriteLkMotorSpeedControlWithTorqueLimit";

    static void
    write(const custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->torque_limit, buf, offset);
        write_int32(msg->speed, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>
{
    static constexpr auto type_enum = "WriteLkMotorMultiRoundPositionControl";

    static void
    write(const custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_int32(msg->angle, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>
{
    static constexpr auto type_enum = "WriteLkMotorMultiRoundPositionControlWithSpeedLimit";

    static void
    write(const custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr& msg, uint8_t* buf,
          int* offset, const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_int16(msg->speed_limit, buf, offset);
        write_int32(msg->angle, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>
{
    static constexpr auto type_enum = "WriteLkMotorSingleRoundPositionControl";

    static void
    write(const custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_uint8(msg->direction, buf, offset);
        write_uint32(msg->angle, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>
{
    static constexpr auto type_enum = "WriteLkMotorSingleRoundPositionControlWithSpeedLimit";

    static void
    write(const custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr& msg, uint8_t* buf,
          int* offset, const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_uint8(msg->direction, buf, offset);
        write_int16(msg->speed_limit, buf, offset);
        write_uint32(msg->angle, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteDJICAN>
{
    static constexpr auto type_enum = "WriteDJICAN";

    static void
    write(const custom_msgs::msg::WriteDJICAN::SharedPtr& msg, uint8_t* buf, int* offset, const std::string& prefix)
    {
        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor1_can_id", prefix)) > 0)
        {
            write_uint8(msg->motor1_enable, buf, offset);
            write_int16(msg->motor1_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor2_can_id", prefix)) > 0)
        {
            write_uint8(msg->motor2_enable, buf, offset);
            write_int16(msg->motor2_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor3_can_id", prefix)) > 0)
        {
            write_uint8(msg->motor3_enable, buf, offset);
            write_int16(msg->motor3_cmd, buf, offset);
        }
        if (get_field_as<uint32_t>(fmt::format("{}sdowrite_motor4_can_id", prefix)) > 0)
        {
            write_uint8(msg->motor4_enable, buf, offset);
            write_int16(msg->motor4_cmd, buf, offset);
        }
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteExternalPWM>
{
    static constexpr auto type_enum = "WriteExternalPWM";

    static void
    write(const custom_msgs::msg::WriteExternalPWM::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& prefix)
    {
        for (int i = 0; i < get_field_as<uint8_t>(fmt::format("{}sdowrite_channel_num", prefix)); i++)
        {
            write_uint16(msg->channels[i], buf, offset);
        }
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteDmMotorMITControl>
{
    static constexpr auto type_enum = "WriteDmMotorMITControl";

    static void
    write(const custom_msgs::msg::WriteDmMotorMITControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& prefix)
    {
        write_uint8(msg->enable, buf, offset);
        static uint8_t WriteDmMotorMITControl_data[8] = {};

        static uint16_t WriteDmMotorMITControl_pos = float_to_uint(msg->p_des,
                                                                   -get_field_as<float>(
                                                                       fmt::format("{}sdowrite_pmax", prefix)),
                                                                   get_field_as<float>(
                                                                       fmt::format("{}sdowrite_pmax", prefix)),
                                                                   16);
        static uint16_t WriteDmMotorMITControl_vel = float_to_uint(msg->v_des,
                                                                   -get_field_as<float>(
                                                                       fmt::format("{}sdowrite_vmax", prefix)),
                                                                   get_field_as<float>(
                                                                       fmt::format("{}sdowrite_vmax", prefix)),
                                                                   12);
        static uint16_t WriteDmMotorMITControl_tor = float_to_uint(msg->torque,
                                                                   -get_field_as<float>(
                                                                       fmt::format("{}sdowrite_tmax", prefix)),
                                                                   get_field_as<float>(
                                                                       fmt::format("{}sdowrite_tmax", prefix)),
                                                                   12);
        static uint16_t WriteDmMotorMITControl_kp = float_to_uint(msg->kp, 0.0, 500.0, 12);
        static uint16_t WriteDmMotorMITControl_kd = float_to_uint(msg->kd, 0.0, 5.0, 12);

        WriteDmMotorMITControl_data[0] = (WriteDmMotorMITControl_pos >> 8);
        WriteDmMotorMITControl_data[1] = WriteDmMotorMITControl_pos;
        WriteDmMotorMITControl_data[2] = (WriteDmMotorMITControl_vel >> 4);
        WriteDmMotorMITControl_data[3] = ((WriteDmMotorMITControl_vel & 0xF) << 4) | (WriteDmMotorMITControl_kp >> 8);
        WriteDmMotorMITControl_data[4] = WriteDmMotorMITControl_kp;
        WriteDmMotorMITControl_data[5] = (WriteDmMotorMITControl_kd >> 4);
        WriteDmMotorMITControl_data[6] = ((WriteDmMotorMITControl_kd & 0xF) << 4) | (WriteDmMotorMITControl_tor >> 8);
        WriteDmMotorMITControl_data[7] = WriteDmMotorMITControl_tor;

        memcpy(buf + *offset, WriteDmMotorMITControl_data, 8);
        *offset += 8;
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>
{
    static constexpr auto type_enum = "WriteDmMotorPositionControlWithSpeedLimit";

    static void
    write(const custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_float(msg->position, buf, offset);
        write_float(msg->speed, buf, offset);
    }
};

template <>
struct MsgDef<custom_msgs::msg::WriteDmMotorSpeedControl>
{
    static constexpr auto type_enum = "WriteDmMotorSpeedControl";

    static void
    write(const custom_msgs::msg::WriteDmMotorSpeedControl::SharedPtr& msg, uint8_t* buf, int* offset,
          const std::string& /*prefix*/)
    {
        write_uint8(msg->enable, buf, offset);
        write_float(msg->speed, buf, offset);
    }
};

#endif //MSG_DEFS_HPP
