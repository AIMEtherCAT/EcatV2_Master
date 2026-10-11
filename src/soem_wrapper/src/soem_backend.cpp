//
// Created by hang on 25-4-21.
//
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

#include "ethercat.h"

namespace aim::ecat {
    using namespace logging;

    static std::vector<std::shared_ptr<device::SlaveDevice> > slave_devices;

    bool init_slave_devices_vector(const int slave_count) {
        slave_devices.clear();
        slave_devices.reserve(slave_count);
        for (int i = 1; i <= slave_count; i++) {
            const device::DeviceRegistry::Entry *entry = get_node()->get_device_registry().find(
                ec_slave[i].eep_man, ec_slave[i].eep_id);
            if (entry == nullptr) {
                RCLCPP_ERROR(*get_cfg_logger(),
                             "Slave id=%d is not registered, vendor=0x%08x, product=0x%08x, name=%s",
                             i,
                             ec_slave[i].eep_man,
                             ec_slave[i].eep_id,
                             ec_slave[i].name);
                slave_devices.clear();
                return false;
            }
            slave_devices.push_back(entry->factory(device::DeviceInfo{
                .index = static_cast<uint16_t>(i),
                .vendor_id = ec_slave[i].eep_man,
                .product_code = ec_slave[i].eep_id,
                .name = entry->name
            }));
            RCLCPP_INFO(*get_cfg_logger(),
                        "Slave id=%d matched device %s, vendor=0x%08x, product=0x%08x",
                        i,
                        entry->name.c_str(),
                        ec_slave[i].eep_man,
                        ec_slave[i].eep_id);
        }
        return true;
    }

    std::shared_ptr<device::SlaveDevice> get_slave_device(const int index) {
        assert(index >= 1);
        return slave_devices.at(index - 1);
    }

    std::vector<std::shared_ptr<device::SlaveDevice> > &get_slave_devices() {
        return slave_devices;
    }

    /**
     * slave sdo configuration write func
     * @param slave slave id
     * @return success or not
     */
    // ReSharper disable once CppParameterMayBeConst
    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave) {
        // called from inside soem, exceptions must not escape
        try {
            return get_slave_device(slave)->configure();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(*get_cfg_logger(), "Slave id=%d configuration failed: %s", slave, e.what());
        } catch (...) {
            RCLCPP_ERROR(*get_cfg_logger(), "Slave id=%d configuration failed", slave);
        }
        return 0;
    }

    static void run() {
        if (get_node()->setup_ecat()) {
            RCLCPP_INFO(*get_sys_logger(), "Initialization succeeded");
            // on a signal, the callback runs in the signal handling thread, while spin may return in the main thread
            // at the same time. the node is held weakly, the callback stays registered in the context after the
            // node is destroyed
            rclcpp::on_shutdown([weak_node = std::weak_ptr(get_node())] {
                if (const auto ecat_node = weak_node.lock()) {
                    ecat_node->on_shutdown();
                }
            });
            rclcpp::spin(get_node());
            get_node()->on_shutdown();
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
