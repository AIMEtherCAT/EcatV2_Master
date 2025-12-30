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
    using namespace sbus_rc;

    custom_msgs::msg::ReadSBUSRC SBUS_RC::custom_msgs_readsbusrc_shared_msg;

    void SBUS_RC::init_sdo(uint8_t * /* buf */, int * /* offset */, const uint16_t slave_id,
                           const std::string &prefix) {
        load_slave_info(slave_id, prefix);

        custom_msgs_readsbusrc_shared_msg.channels.resize(16);

        publisher_ = get_node()->create_publisher<custom_msgs::msg::ReadSBUSRC>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}pub_topic", prefix)),
            rclcpp::SensorDataQoS()
        );
    }

    void SBUS_RC::publish_empty_message() {
        custom_msgs_readsbusrc_shared_msg.header.stamp = rclcpp::Clock().now();

        for (int i = 0; i < 16; i++) {
            custom_msgs_readsbusrc_shared_msg.channels[i] = 0;
        }

        custom_msgs_readsbusrc_shared_msg.ch17 = 0;
        custom_msgs_readsbusrc_shared_msg.ch18 = 0;
        custom_msgs_readsbusrc_shared_msg.fail_safe = 0;
        custom_msgs_readsbusrc_shared_msg.frame_lost = 0;

        custom_msgs_readsbusrc_shared_msg.online = 0;
        publisher_->publish(custom_msgs_readsbusrc_shared_msg);
    }

    void SBUS_RC::read() {
        custom_msgs_readsbusrc_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;

        if (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 23]) {
            custom_msgs_readsbusrc_shared_msg.channels[0] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 0] | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 1] << 8) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[1] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 1] >> 3 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 2] << 5) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[2] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 2] >> 6 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 3] << 2 | slave_device_->get_slave_to_master_buf()[
                 pdoread_offset_ + 4] << 10) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[3] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 4] >> 1 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 5] << 7) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[4] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 5] >> 4 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 6] << 4) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[5] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 6] >> 7 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 7] << 1 | slave_device_->get_slave_to_master_buf()[
                 pdoread_offset_ + 8] << 9) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[6] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 8] >> 2 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 9] << 6) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[7] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 9] >> 5 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 10] << 3) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[8] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 11] | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 12] << 8) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[9] =
                    (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 12] >> 3 | slave_device_->
                     get_slave_to_master_buf()[pdoread_offset_ + 13] << 5) &
                    0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[10] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 13] >> 6 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 14] << 2 | slave_device_->get_slave_to_master_buf()[
                 pdoread_offset_ + 15] << 10) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[11] =
                    (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 15] >> 1 | slave_device_->
                     get_slave_to_master_buf()[pdoread_offset_ + 16] << 7) &
                    0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[12] =
                    (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 16] >> 4 | slave_device_->
                     get_slave_to_master_buf()[pdoread_offset_ + 17] << 4) &
                    0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[13] =
            (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 17] >> 7 | slave_device_->
             get_slave_to_master_buf()[pdoread_offset_ + 18] << 1 | slave_device_->get_slave_to_master_buf()[
                 pdoread_offset_ + 19] << 9) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[14] =
                    (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 19] >> 2 | slave_device_->
                     get_slave_to_master_buf()[pdoread_offset_ + 20] << 6) &
                    0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[15] =
                    (slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 20] >> 5 | slave_device_->
                     get_slave_to_master_buf()[pdoread_offset_ + 21] << 3) &
                    0x07FF;

            custom_msgs_readsbusrc_shared_msg.ch17 =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 22] & 0x01 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.ch18 =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 22] & 0x02 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.fail_safe =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 22] & 0x04 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.frame_lost =
                    slave_device_->get_slave_to_master_buf()[pdoread_offset_ + 22] & 0x08 ? 1 : 0;

            custom_msgs_readsbusrc_shared_msg.online = 1;
        } else {
            custom_msgs_readsbusrc_shared_msg.online = 0;
        }
        publisher_->publish(custom_msgs_readsbusrc_shared_msg);
    }
}
