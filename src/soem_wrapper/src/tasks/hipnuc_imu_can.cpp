//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task::hipnuc_imu {
    using namespace aim::io::little_endian;
    using namespace utils::dynamic_data;

    sensor_msgs::msg::Imu HIPNUC_IMU_CAN::sensor_msgs_imu_shared_msg;

    void HIPNUC_IMU_CAN::init_sdo(uint8_t *buf, int *offset, const uint32_t /*sn*/, const uint8_t /*slave_id*/,
                                  const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                {"can_inst", "packet1_id", "packet2_id", "packet3_id"});
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;
        get_node()->create_and_insert_publisher<sensor_msgs::msg::Imu>(prefix);
    }

    void HIPNUC_IMU_CAN::publish_empty_message(const std::string &prefix) {
        sensor_msgs_imu_shared_msg.header.stamp = rclcpp::Clock().now();
        sensor_msgs_imu_shared_msg.header.frame_id = get_field_as<std::string>(
            *get_dynamic_data(),
            fmt::format("{}conf_frame_name", prefix),
            "imu_link");

        // hipnuc hi92 protocol
        sensor_msgs_imu_shared_msg.orientation.w = 1;
        sensor_msgs_imu_shared_msg.orientation.x = 0;
        sensor_msgs_imu_shared_msg.orientation.y = 0;
        sensor_msgs_imu_shared_msg.orientation.z = 0;

        sensor_msgs_imu_shared_msg.linear_acceleration.x = 0;
        sensor_msgs_imu_shared_msg.linear_acceleration.y = 0;
        sensor_msgs_imu_shared_msg.linear_acceleration.z = 0;

        sensor_msgs_imu_shared_msg.angular_velocity.x = 0;
        sensor_msgs_imu_shared_msg.angular_velocity.y = 0;
        sensor_msgs_imu_shared_msg.angular_velocity.z = 0;

        EthercatNode::publish_msg<sensor_msgs::msg::Imu>(prefix, sensor_msgs_imu_shared_msg);
    }

    void HIPNUC_IMU_CAN::read(const uint8_t *buf, int *offset, const std::string &prefix) {
        sensor_msgs_imu_shared_msg.header.stamp = rclcpp::Clock().now();
        sensor_msgs_imu_shared_msg.header.frame_id = get_field_as<std::string>(
            *get_dynamic_data(),
            fmt::format("{}conf_frame_name", prefix),
            "imu_link");

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
}
