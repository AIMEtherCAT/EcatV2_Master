//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"

namespace aim::ecat::task {
    using namespace aim::io::little_endian;
    using namespace utils::dynamic_data;

    using namespace pwm;

    void ONBOARD_PWM::init_sdo(uint8_t *buf, int *offset, const uint32_t /*sn*/, const uint8_t slave_id,
                               const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                {
                                                                    "connection_lost_write_action", "port_id",
                                                                    "pwm_period", "init_value"
                                                                });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;
        get_node()->create_and_insert_subscriber<custom_msgs::msg::WriteOnBoardPWM>(prefix, slave_id);
    }

    void ONBOARD_PWM::init_value(uint8_t *buf, int *offset, const std::string &prefix) {
        const uint16_t init_value = get_field_as<uint16_t>(*get_dynamic_data(),
                                                           fmt::format("{}sdowrite_init_value", prefix));
        for (int i = 1; i <= 4; i++) {
            RCLCPP_INFO(*logging::get_cfg_logger(), "prefix=%s will write init value=%d at m2s buf idx=%d",
                        prefix.c_str(),
                        init_value,
                        *offset);
            write_uint16(init_value, buf, offset);
        }
    }

    void MsgDef<custom_msgs::msg::WriteOnBoardPWM>::write(const custom_msgs::msg::WriteOnBoardPWM::SharedPtr &msg,
                                                          uint8_t *buf, int *offset,
                                                          const std::string & /*prefix*/) {
        write_uint16(msg->channel1, buf, offset);
        write_uint16(msg->channel2, buf, offset);
        write_uint16(msg->channel3, buf, offset);
        write_uint16(msg->channel4, buf, offset);
    }
}
