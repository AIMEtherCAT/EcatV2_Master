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
    using namespace hipnuc_imu;

    sensor_msgs::msg::Imu HIPNUC_IMU_CAN::sensor_msgs_imu_shared_msg;

    void HIPNUC_IMU_CAN::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id, const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                      {
                                                                          "can_inst", "packet1_id", "packet2_id",
                                                                          "packet3_id"
                                                                      });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        load_slave_info(slave_id, prefix);

        conf_frame_name_ = get_field_as<std::string>(
            *get_configuration_data(),
            fmt::format("{}conf_frame_name", prefix),
            "imu_link");

        publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void HIPNUC_IMU_CAN::publish_empty_message() {
        sensor_msgs_imu_shared_msg.header.stamp = rclcpp::Clock().now();
        sensor_msgs_imu_shared_msg.header.frame_id = conf_frame_name_;

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

        publisher_->publish(sensor_msgs_imu_shared_msg);
    }

    void HIPNUC_IMU_CAN::read() {
        sensor_msgs_imu_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;
        sensor_msgs_imu_shared_msg.header.frame_id = conf_frame_name_;

        int offset = pdoread_offset_;

        // hipnuc hi92 protocol
        sensor_msgs_imu_shared_msg.orientation.w = 0.0001 * read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                       &offset);
        sensor_msgs_imu_shared_msg.orientation.x = 0.0001 * read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                       &offset);
        sensor_msgs_imu_shared_msg.orientation.y = 0.0001 * read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                       &offset);
        sensor_msgs_imu_shared_msg.orientation.z = 0.0001 * read_int16(slave_device_->get_slave_to_master_buf().data(),
                                                                       &offset);

        sensor_msgs_imu_shared_msg.linear_acceleration.x = 0.0048828 * read_int16(
                                                               slave_device_->get_slave_to_master_buf().data(),
                                                               &offset);
        sensor_msgs_imu_shared_msg.linear_acceleration.y = 0.0048828 * read_int16(
                                                               slave_device_->get_slave_to_master_buf().data(),
                                                               &offset);
        sensor_msgs_imu_shared_msg.linear_acceleration.z = 0.0048828 * read_int16(
                                                               slave_device_->get_slave_to_master_buf().data(),
                                                               &offset);

        sensor_msgs_imu_shared_msg.angular_velocity.x = 0.001 * read_int16(
                                                            slave_device_->get_slave_to_master_buf().data(), &offset);
        sensor_msgs_imu_shared_msg.angular_velocity.y = 0.001 * read_int16(
                                                            slave_device_->get_slave_to_master_buf().data(), &offset);
        sensor_msgs_imu_shared_msg.angular_velocity.z = 0.001 * read_int16(
                                                            slave_device_->get_slave_to_master_buf().data(), &offset);

        publisher_->publish(sensor_msgs_imu_shared_msg);
    }
}
