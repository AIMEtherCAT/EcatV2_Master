//
// Created by hang on 25-6-27.
//

#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "soem_wrapper/utils/io_utils.hpp"
#include "soem_wrapper/dynamic_data_helper.hpp"
#include "soem_wrapper/defs/task_defs.hpp"

#include "ethercattype.h"
#include "nicdrv.h"
#include "osal.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatdc.h"

namespace aim::ecat {
    using namespace aim::utils::dynamic_data;
    using namespace aim::io::little_endian;
    using namespace std::chrono_literals;

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

        template<typename MsgT>
        void create_and_insert_publisher(const std::string &prefix);

        template<typename MsgT>
        static void publish_msg(const std::string &prefix, const MsgT &msg);

        template<typename MsgT>
        void create_and_insert_subscriber(const std::string &prefix, uint8_t slave_id);

        /**
         * ros2 msg subscriber callback func
         * write data to corresponding PDO buffer
         * @tparam MsgT ros2 msg type
         * @param msg msg ref
         * @param prefix dynamic struct key prefix
         * @param slave_id slave id
         */
        template<typename MsgT>
        void generic_callback(const MsgT::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    private:
        void register_components();

        void register_module(uint32_t eep_id,
                             const std::string &module_name,
                             int master_to_slave_buf_len,
                             int slave_to_master_buf_len,
                             int min_sw_rev);

        void register_app(task::TaskWrapper *task_wrapper);


        void datacycle_callback();

        void state_check_callback() const;

        char IOmap[4096]{};

        std::string interface{};
        int rt_cpu{};
        std::string non_rt_cpus{};
        std::string config_file{};

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

        int expectedWkc{};
        std::atomic<int> wkc{};
        std::atomic<bool> in_operational_{};
    };


    std::shared_ptr<EthercatNode> get_node();


    void register_node();

    void destroy_node();
}

#endif //ETHERCAT_NODE_HPP
