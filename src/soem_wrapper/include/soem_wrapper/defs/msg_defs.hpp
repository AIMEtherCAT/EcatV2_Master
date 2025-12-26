//
// Created by hang on 25-6-25.
//

#ifndef MSG_DEFS_HPP
#define MSG_DEFS_HPP

#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat::task {
    template<typename MsgT>
    struct MsgDef {
        static constexpr auto type_enum = "Unknown";

        static void
        write(const MsgT::SharedPtr & /*msg*/, uint8_t * /*buf*/, int * /*offset*/,
              const std::string &prefix) {
            RCLCPP_ERROR(*logging::get_data_logger(), "Unsupported MsgT in generic_callback for prefix: %s", prefix.c_str());
        }
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteDSHOT> {
        static constexpr auto type_enum = "WriteDSHOT";

        static void
        write(const custom_msgs::msg::WriteDSHOT::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteOnBoardPWM> {
        static constexpr auto type_enum = "WriteOnBoardPWM";

        static void
        write(const custom_msgs::msg::WriteOnBoardPWM::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorOpenloopControl> {
        static constexpr auto type_enum = "WriteLkMotorOpenloopControl";

        static void
        write(const custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorTorqueControl> {
        static constexpr auto type_enum = "WriteLkMotorTorqueControl";

        static void
        write(const custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit> {
        static constexpr auto type_enum = "WriteLkMotorSpeedControlWithTorqueLimit";

        static void
        write(const custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr &msg, uint8_t *buf,
              int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl> {
        static constexpr auto type_enum = "WriteLkMotorMultiRoundPositionControl";

        static void
        write(const custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit> {
        static constexpr auto type_enum = "WriteLkMotorMultiRoundPositionControlWithSpeedLimit";

        static void
        write(const custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr &msg, uint8_t *buf,
              int *offset, const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl> {
        static constexpr auto type_enum = "WriteLkMotorSingleRoundPositionControl";

        static void
        write(const custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit> {
        static constexpr auto type_enum = "WriteLkMotorSingleRoundPositionControlWithSpeedLimit";

        static void
        write(const custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr &msg,
              uint8_t *buf,
              int *offset, const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteDJIMotor> {
        static constexpr auto type_enum = "WriteDJIMotor";

        static void
        write(const custom_msgs::msg::WriteDJIMotor::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string &prefix);
    };

    // TODO: task waiting for test
    // template<>
    // struct MsgDef<custom_msgs::msg::WriteExternalPWM> {
    //     static constexpr auto type_enum = "WriteExternalPWM";
    //
    //     static void
    //     write(const custom_msgs::msg::WriteExternalPWM::SharedPtr &msg, uint8_t *buf, int *offset,
    //           const std::string &prefix) {
    //         for (int i = 0; i < get_field_as<uint8_t>(fmt::format("{}sdowrite_channel_num", prefix)); i++) {
    //             write_uint16(msg->channels[i], buf, offset);
    //         }
    //     }
    // };

    template<>
    struct MsgDef<custom_msgs::msg::WriteDmMotorMITControl> {
        static constexpr auto type_enum = "WriteDmMotorMITControl";

        static void
        write(const custom_msgs::msg::WriteDmMotorMITControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string &prefix);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit> {
        static constexpr auto type_enum = "WriteDmMotorPositionControlWithSpeedLimit";

        static void
        write(const custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit::SharedPtr &msg, uint8_t *buf,
              int *offset,
              const std::string & /*prefix*/);
    };

    template<>
    struct MsgDef<custom_msgs::msg::WriteDmMotorSpeedControl> {
        static constexpr auto type_enum = "WriteDmMotorSpeedControl";

        static void
        write(const custom_msgs::msg::WriteDmMotorSpeedControl::SharedPtr &msg, uint8_t *buf, int *offset,
              const std::string & /*prefix*/);
    };
}

#endif //MSG_DEFS_HPP
