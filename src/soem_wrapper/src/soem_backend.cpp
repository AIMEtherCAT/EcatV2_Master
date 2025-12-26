//
// Created by hang on 25-4-21.
//
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

#include "ethercattype.h"
#include "osal.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"

#include "mutex"

namespace aim::ecat {
    using namespace logging;
    
    SlaveDeviceT slave_devices[EC_MAXSLAVE]{};

    SlaveDeviceT *get_slave_devices() {
        return slave_devices;
    }

    int sdo_sn_size_ptr = 4;
    int sdo_rev_size_ptr = 3;
    uint16_t sdo_size_write_ptr = 0;

    /**
     * slave sdo configuration write func
     * @param slave slave id
     * @return success or not
     */
    // ReSharper disable once CppParameterMayBeConst
    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave) {
        try {
            std::lock_guard lock(slave_devices[slave].mtx);
            const uint32_t oldSn = slave_devices[slave].sn;
            // read device info
            sdo_sn_size_ptr = 4;
            ec_SDOread(slave, 0x1018, 4, FALSE, &sdo_sn_size_ptr, &slave_devices[slave].sn, 0xffff);
            sdo_rev_size_ptr = 3;
            ec_SDOread(slave, 0x100A, 0, FALSE, &sdo_rev_size_ptr, &slave_devices[slave].sw_rev_str, 0xffff);
            slave_devices[slave].sw_rev = atoi(slave_devices[slave].sw_rev_str); // NOLINT

            if (oldSn != 0) {
                RCLCPP_INFO_THROTTLE(*get_cfg_logger(),
                                     *get_node()->get_clock(), 1500,
                                     "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                                     slave,
                                     slave_devices[slave].sn,
                                     ec_slave[slave].eep_id,
                                     get_node()->get_device_name(ec_slave[slave].eep_id).c_str(),
                                     slave_devices[slave].sw_rev);
            } else {
                RCLCPP_INFO(*get_cfg_logger(),
                            "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                            slave,
                            slave_devices[slave].sn,
                            ec_slave[slave].eep_id,
                            get_node()->get_device_name(ec_slave[slave].eep_id).c_str(),
                            slave_devices[slave].sw_rev);
            }

            if (oldSn != 0 && oldSn != slave_devices[slave].sn) {
                RCLCPP_ERROR_THROTTLE(*get_cfg_logger(),
                                      *get_node()->get_clock(),
                                      1500,
                                      "Slave idx=%d reconnected with different board, config rejected",
                                      slave);
                slave_devices[slave].sn = oldSn;
                slave_devices[slave].recover_rejected = 1;
                return 0;
            }
            slave_devices[slave].recover_rejected = 0;

            if (slave_devices[slave].sw_rev < get_node()->get_device_min_sw_rev_requirement(ec_slave[slave].eep_id)) {
                RCLCPP_ERROR(
                    *get_cfg_logger(),
                    "Slave id=%d, sn=%d, swrev=%d, don't meet the requirement of minimum sw rev=%d, please flash the newest firmware.",
                    slave, slave_devices[slave].sn,
                    slave_devices[slave].sw_rev,
                    get_node()->get_device_min_sw_rev_requirement(ec_slave[slave].eep_id));
                return 0;
            }
            slave_devices[slave].sw_rev_check_passed = 1;

            // write device sdo len
            sdo_size_write_ptr = get_field_as<uint16_t>(
                *get_dynamic_data(),
                slave_devices[slave].sn,
                "sdo_len");
            ec_SDOwrite(slave, 0x8000, 0, FALSE, 2, &sdo_size_write_ptr, 0xffff);

            // init buffers
            slave_devices[slave].device_type = ec_slave[slave].eep_id;
            slave_devices[slave].master_status = sdo_size_write_ptr == 0 ? MASTER_READY : MASTER_SENDING_ARGUMENTS;

            // only explicitly clear buf in first configuration try
            if (oldSn == 0) {
                slave_devices[slave].master_to_slave_buf.clear();
                slave_devices[slave].master_to_slave_buf.resize(
                    get_node()->get_device_master_to_slave_buf_len(ec_slave[slave].eep_id));
                slave_devices[slave].master_to_slave_buf_len = get_node()->get_device_master_to_slave_buf_len(
                    ec_slave[slave].eep_id);
                slave_devices[slave].master_to_slave_buf.clear();

                slave_devices[slave].slave_to_master_buf.clear();
                slave_devices[slave].slave_to_master_buf.resize(
                    get_node()->get_device_slave_to_master_buf_len(ec_slave[slave].eep_id));
                slave_devices[slave].slave_to_master_buf_len = get_node()->get_device_slave_to_master_buf_len(
                    ec_slave[slave].eep_id);
                slave_devices[slave].slave_to_master_buf.clear();
            }

            RCLCPP_INFO(*get_cfg_logger(),
                        "SDO configured for slave id=%d, sn=%d, eepid=%d, type=%s, sdolen=%d",
                        slave,
                        slave_devices[slave].sn,
                        ec_slave[slave].eep_id,
                        get_node()->get_device_name(ec_slave[slave].eep_id).c_str(), sdo_size_write_ptr);

            slave_devices[slave].conf_done = 1;
            return 1;
        } catch (...) {
            return 0;
        }
    }

    void run() {
        if (get_node()->setup_ecat()) {
            RCLCPP_INFO(*get_sys_logger(), "Initialization succeeded");
            rclcpp::spin(get_node());
        } else {
            RCLCPP_ERROR(*get_sys_logger(), "Initialization failed");
        }
    }
}

int main(const int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    aim::ecat::register_node();
    aim::ecat::run();
    aim::ecat::destroy_node();
    rclcpp::shutdown();
    return 0;
}
