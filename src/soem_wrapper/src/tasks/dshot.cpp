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
    using namespace pwm;

    void DSHOT::init_sdo(uint8_t *buf, int *offset, const uint16_t slave_id, const std::string &prefix) {
        auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(fmt::format("{}sdowrite_", prefix),
                                                                      {
                                                                          "connection_lost_write_action", "dshot_id",
                                                                          "init_value"
                                                                      });
        memcpy(buf + *offset, sdo_buf, sdo_len);
        *offset += sdo_len;

        load_slave_info(slave_id, prefix);

        init_value_ = get_field_as<uint16_t>(*get_configuration_data(),
                                             fmt::format("{}sdowrite_init_value", prefix));

        subscriber_ = get_node()->create_subscription<custom_msgs::msg::WriteDSHOT>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("{}sub_topic", prefix)),
            rclcpp::SensorDataQoS(),
            std::bind(&DSHOT::on_command, this, std::placeholders::_1)
        );
    }

    void DSHOT::init_value() {
        int offset = pdowrite_offset_;

        for (int i = 1; i <= 4; i++) {
            write_uint16(init_value_, slave_device_->get_master_to_slave_buf().data(), &offset);
        }
    }

    void DSHOT::on_command(const custom_msgs::msg::WriteDSHOT::SharedPtr msg) const {
        std::lock_guard lock(slave_device_->mtx_);
        int offset = pdowrite_offset_;

        write_uint16(msg->channel1, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint16(msg->channel2, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint16(msg->channel3, slave_device_->get_master_to_slave_buf().data(), &offset);
        write_uint16(msg->channel4, slave_device_->get_master_to_slave_buf().data(), &offset);
    }
}
