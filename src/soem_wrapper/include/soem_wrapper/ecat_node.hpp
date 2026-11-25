//
// Created by hang on 25-6-27.
//

#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "chrono"
#include "mutex"
#include "thread"
#include "rclcpp/rclcpp.hpp"

#include "soem_wrapper/device/device_registry.hpp"
#include "soem_wrapper/sdo_worker.hpp"

#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif

namespace aim::ecat {
    class EthercatNode final : public rclcpp::Node {
    public:
        EthercatNode();

        ~EthercatNode() override;

        bool setup_ecat();

        void on_shutdown();

        [[nodiscard]] const device::DeviceRegistry &get_device_registry() const {
            return device_registry_;
        }

        /**
         * global worker for blocking sdo operations at runtime, see SdoWorker
         */
        SdoWorker &get_sdo_worker() {
            return sdo_worker_;
        }

    private:
        void register_components();

        /**
         * register a device implementation for the given vendor id and product code
         * extra args are forwarded to the constructor of T after the DeviceInfo
         */
        template<typename T, typename... Args>
        void register_device(const uint32_t vendor_id,
                             const uint32_t product_code,
                             const std::string &name,
                             Args... args) {
            device_registry_.add(vendor_id, product_code, name,
                                 [=](const device::DeviceInfo &info) -> std::shared_ptr<device::SlaveDevice> {
                                     return std::make_shared<T>(info, args...);
                                 });
        }

        void do_shutdown();

        void datacycle_callback();

        void state_check_callback();

        char IOmap_[4096]{};

        std::string interface_{};
        int rt_cpu_{};
        std::string non_rt_cpus_{};
        std::string config_file_{};

        device::DeviceRegistry device_registry_{};
        rclcpp::Clock data_clock_{};

        std::once_flag shutdown_once_{};
        std::atomic<bool> running_{};
        std::atomic<bool> exiting_{};
        std::atomic<bool> exiting_reset_called_{};
        std::thread data_thread_{};
        std::thread checker_thread_{};
        SdoWorker sdo_worker_{};

        int expectedWkc_{};
        std::atomic<int> wkc_{};
        std::atomic<bool> in_operational_{};
    };

    std::shared_ptr<EthercatNode> get_node();

    void register_node();

    void destroy_node();
}

#endif //ETHERCAT_NODE_HPP
