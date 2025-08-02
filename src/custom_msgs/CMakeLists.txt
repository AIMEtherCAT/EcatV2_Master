cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ReadDJIRC.msg"

    "msg/ReadSBUSRC.msg"

    "msg/WriteDSHOT.msg"

    "msg/WriteDJICAN.msg"
    "msg/ReadDJICAN.msg"

    "msg/WriteLkMotorOpenloopControl.msg"
    "msg/WriteLkMotorTorqueControl.msg"
    "msg/WriteLkMotorSpeedControlWithTorqueLimit.msg"
    "msg/WriteLkMotorMultiRoundPositionControl.msg"
    "msg/WriteLkMotorMultiRoundPositionControlWithSpeedLimit.msg"
    "msg/WriteLkMotorSingleRoundPositionControl.msg"
    "msg/WriteLkMotorSingleRoundPositionControlWithSpeedLimit.msg"
    "msg/ReadLkMotor.msg"

    "msg/WriteVanillaPWM.msg"
    "msg/WriteExternalPWM.msg"

    "msg/ReadMS5876BA30.msg"

    "msg/ReadADC.msg"

    "msg/ReadCANPMU.msg"

    DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_package()
