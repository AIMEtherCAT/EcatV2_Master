//
// Created by hang on 25-6-27.
//

#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "chrono"
#include "thread"
#include "rclcpp/rclcpp.hpp"

namespace aim::ecat::task {
    class TaskWrapper;
}

namespace aim::ecat {
    // slave2master status enums
    constexpr uint8_t SLAVE_INITIALIZING = 1;
    constexpr uint8_t SLAVE_READY = 2;
    constexpr uint8_t SLAVE_CONFIRM_READY = 3;

    // master2slave status enums
    constexpr uint8_t MASTER_REQUEST_REBOOT = 1;
    constexpr uint8_t MASTER_SENDING_ARGUMENTS = 2;
    constexpr uint8_t MASTER_READY = 3;


    class EthercatNode final : public rclcpp::Node {
    public:
        EthercatNode();

        ~EthercatNode() override;

        bool setup_ecat();

        void on_shutdown();

        std::string get_device_name(const uint32_t eep_id) {
            return registered_module_names[eep_id];
        }

        int get_device_min_sw_rev_requirement(const uint32_t eep_id) {
            return registered_module_sw_rev[eep_id];
        }

        uint16_t get_device_master_to_slave_buf_len(const uint32_t eep_id) {
            return registered_module_buf_lens[eep_id].first;
        }

        uint16_t get_device_slave_to_master_buf_len(const uint32_t eep_id) {
            return registered_module_buf_lens[eep_id].second;
        }

    private:
        void register_components();

        void register_module(uint32_t eep_id,
                             const std::string &module_name,
                             int master_to_slave_buf_len,
                             int slave_to_master_buf_len,
                             int min_sw_rev);

        void datacycle_callback();

        void state_check_callback() const;

        char IOmap_[4096]{};

        std::string interface_{};
        int rt_cpu_{};
        std::string non_rt_cpus_{};
        std::string config_file_{};

        // m2s, s2m
        std::unordered_map<uint32_t, std::pair<uint16_t, uint16_t> > registered_module_buf_lens{};
        std::unordered_map<uint32_t, std::string> registered_module_names{};
        std::unordered_map<uint32_t, int> registered_module_sw_rev{};
        std::unordered_map<uint8_t, task::TaskWrapper *> app_registry{};

        std::atomic<bool> running_{};
        std::atomic<bool> exiting_{};
        std::atomic<bool> exiting_reset_called_{};
        std::thread data_thread_{};
        std::thread checker_thread_{};

        int expectedWkc_{};
        std::atomic<int> wkc_{};
        std::atomic<bool> in_operational_{};
    };

    std::shared_ptr<EthercatNode> get_node();

    void register_node();

    void destroy_node();
}

#endif //ETHERCAT_NODE_HPP