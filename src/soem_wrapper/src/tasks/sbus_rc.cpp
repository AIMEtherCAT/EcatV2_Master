//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task::sbus_rc {
    using namespace aim::io::little_endian;
    using namespace utils::dynamic_data;

    custom_msgs::msg::ReadSBUSRC SBUS_RC::custom_msgs_readsbusrc_shared_msg;

    void SBUS_RC::init_sdo(uint8_t * /*buf*/, int * /*offset*/, const uint32_t /*sn*/, const uint8_t /*slave_id*/,
                          const std::string &prefix) {
        get_node()->create_and_insert_publisher<custom_msgs::msg::ReadSBUSRC>(prefix);
        custom_msgs_readsbusrc_shared_msg.channels.resize(16);
    }

    void SBUS_RC::publish_empty_message(const std::string &prefix) {
        custom_msgs_readsbusrc_shared_msg.header.stamp = rclcpp::Clock().now();

        for (int i = 0; i < 16; i++) {
            custom_msgs_readsbusrc_shared_msg.channels[i] = 0;
        }

        custom_msgs_readsbusrc_shared_msg.ch17 = 0;
        custom_msgs_readsbusrc_shared_msg.ch18 = 0;
        custom_msgs_readsbusrc_shared_msg.fail_safe = 0;
        custom_msgs_readsbusrc_shared_msg.frame_lost = 0;

        custom_msgs_readsbusrc_shared_msg.online = 0;
        EthercatNode::publish_msg<custom_msgs::msg::ReadSBUSRC>(prefix, custom_msgs_readsbusrc_shared_msg);
    }

    void SBUS_RC::read(const uint8_t *buf, int *offset, const std::string &prefix) { // NOLINT
        custom_msgs_readsbusrc_shared_msg.header.stamp = rclcpp::Clock().now();

        if (buf[*offset + 23]) {
            custom_msgs_readsbusrc_shared_msg.channels[0] = (buf[*offset + 0] | buf[*offset + 1] << 8) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[1] =
                    (buf[*offset + 1] >> 3 | buf[*offset + 2] << 5) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[2] = (buf[*offset + 2] >> 6 | buf[*offset + 3] << 2 | buf[*
                                                                 offset + 4] << 10) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[3] =
                    (buf[*offset + 4] >> 1 | buf[*offset + 5] << 7) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[4] =
                    (buf[*offset + 5] >> 4 | buf[*offset + 6] << 4) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[5] = (buf[*offset + 6] >> 7 | buf[*offset + 7] << 1 | buf[*
                                                                 offset + 8] << 9) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[6] =
                    (buf[*offset + 8] >> 2 | buf[*offset + 9] << 6) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[7] =
                    (buf[*offset + 9] >> 5 | buf[*offset + 10] << 3) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[8] = (buf[*offset + 11] | buf[*offset + 12] << 8) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[9] = (buf[*offset + 12] >> 3 | buf[*offset + 13] << 5) &
                                                            0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[10] =
            (buf[*offset + 13] >> 6 | buf[*offset + 14] << 2 | buf[*
                                                                   offset + 15] << 10) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[11] = (buf[*offset + 15] >> 1 | buf[*offset + 16] << 7) &
                                                             0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[12] = (buf[*offset + 16] >> 4 | buf[*offset + 17] << 4) &
                                                             0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[13] =
            (buf[*offset + 17] >> 7 | buf[*offset + 18] << 1 | buf[*
                                                                   offset + 19] << 9) & 0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[14] = (buf[*offset + 19] >> 2 | buf[*offset + 20] << 6) &
                                                             0x07FF;
            custom_msgs_readsbusrc_shared_msg.channels[15] = (buf[*offset + 20] >> 5 | buf[*offset + 21] << 3) &
                                                             0x07FF;

            custom_msgs_readsbusrc_shared_msg.ch17 = buf[*offset + 22] & 0x01 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.ch18 = buf[*offset + 22] & 0x02 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.fail_safe = buf[*offset + 22] & 0x04 ? 1 : 0;
            custom_msgs_readsbusrc_shared_msg.frame_lost = buf[*offset + 22] & 0x08 ? 1 : 0;

            custom_msgs_readsbusrc_shared_msg.online = 1;
        } else {
            custom_msgs_readsbusrc_shared_msg.online = 0;
        }
        EthercatNode::publish_msg<custom_msgs::msg::ReadSBUSRC>(prefix, custom_msgs_readsbusrc_shared_msg);
    }
}
