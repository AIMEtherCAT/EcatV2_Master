//
// Created by hang on 10/10/26.
//
#include "soem_wrapper/device/slave_device.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

#include "ethercat.h"

namespace aim::ecat::device {
    SlaveDevice::SlaveDevice(DeviceInfo info) : info_(std::move(info)) {
        config_prefix_ = fmt::format("slave{}_", info_.index);
    }

    int SlaveDevice::configure() {
        ConfigContext ctx{
            .is_reconfigure = configured_once_,
            .config = *utils::config::get_configuration_data()
        };
        const int result = on_configure(ctx);
        configured_once_ = true;
        return result;
    }

    void SlaveDevice::post_map() {
        rx_buf_.assign(input_size(), 0);
        tx_buf_.assign(output_size(), 0);
    }

    bool SlaveDevice::setup(SetupContext &ctx) {
        try {
            setup_done_ = on_setup(ctx);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(*logging::get_cfg_logger(),
                         "Slave id=%d (%s) setup failed with exception: %s",
                         info_.index,
                         info_.name.c_str(),
                         e.what());
            setup_done_ = false;
        } catch (...) {
            RCLCPP_ERROR(*logging::get_cfg_logger(),
                         "Slave id=%d (%s) setup failed with unknown exception",
                         info_.index,
                         info_.name.c_str());
            setup_done_ = false;
        }
        return setup_done_;
    }

    int SlaveDevice::on_configure(ConfigContext & /* ctx */) {
        return 1;
    }

    bool SlaveDevice::on_setup(SetupContext & /* ctx */) {
        return true;
    }

    void SlaveDevice::on_op_reached() {
        ready_ = true;
    }

    void SlaveDevice::on_receive() {
        if (!rx_buf_.empty() && ec_slave[info_.index].inputs != nullptr) {
            memcpy(rx_buf_.data(), ec_slave[info_.index].inputs, rx_buf_.size());
        }
    }

    void SlaveDevice::on_cycle(const CycleContext & /* ctx */) {
    }

    void SlaveDevice::on_transmit() {
        if (!tx_buf_.empty() && ec_slave[info_.index].outputs != nullptr) {
            memcpy(ec_slave[info_.index].outputs, tx_buf_.data(), tx_buf_.size());
        }
    }

    void SlaveDevice::write_safe_outputs() {
        std::ranges::fill(tx_buf_, 0);
    }

    bool SlaveDevice::is_configured() const {
        return setup_done_;
    }

    bool SlaveDevice::is_ready() const {
        return ready_;
    }

    void SlaveDevice::pre_recover() {
        ready_ = true;
    }

    void SlaveDevice::reset_state() {
        ready_ = false;
    }

    void SlaveDevice::recover_state() {
        // ec_recover_slave only restores the station address, the slave is still in INIT
        // so only mark ready if it is really operational
        if (is_operational()) {
            ready_ = true;
        }
    }

    void SlaveDevice::on_connection_lost() {
        write_safe_outputs();
    }

    bool SlaveDevice::is_recover_rejected() const {
        return false;
    }

    void SlaveDevice::cleanup() {
    }

    uint32_t SlaveDevice::input_size() const {
        return ec_slave[info_.index].Ibytes;
    }

    uint32_t SlaveDevice::output_size() const {
        return ec_slave[info_.index].Obytes;
    }

    bool SlaveDevice::is_operational() const {
        return ec_slave[info_.index].state == EC_STATE_OPERATIONAL;
    }

    bool SlaveDevice::sdo_read_raw(const uint16_t index, const uint8_t sub_index, void *buf, const int size) const {
        int read_size = size;
        const int wkc = ec_SDOread(info_.index, index, sub_index, FALSE, &read_size, buf, EC_TIMEOUTRXM);
        return wkc > 0 && read_size == size;
    }

    bool SlaveDevice::sdo_write_raw(const uint16_t index, const uint8_t sub_index, void *buf, const int size) const {
        return ec_SDOwrite(info_.index, index, sub_index, FALSE, size, buf, EC_TIMEOUTRXM) > 0;
    }
}
