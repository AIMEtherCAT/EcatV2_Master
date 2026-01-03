//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/utils/io_utils.hpp"
#include "soem_wrapper/utils/sys_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"
#include "soem_wrapper/utils/config_utils.hpp"

#include "ethercat.h"

namespace aim::ecat {
    using namespace aim::utils::config;
    using namespace aim::io::little_endian;
    using namespace std::chrono_literals;

    static std::shared_ptr<EthercatNode> node = nullptr;

    void register_node() {
        node = std::make_shared<EthercatNode>();
    }

    std::shared_ptr<EthercatNode> get_node() {
        return node;
    }

    void destroy_node() {
        node.reset();
    }

    EthercatNode::EthercatNode() : Node("EthercatNode"), running_(true), exiting_(false), exiting_reset_called_(false) {
        this->declare_parameter<std::string>("interface", "enp2s0");
        interface_ = this->get_parameter("interface").as_string();
        RCLCPP_INFO(*logging::get_sys_logger(), "Using interface: %s", interface_.c_str());

        this->declare_parameter<int>("rt_cpu", 6);
        rt_cpu_ = this->get_parameter("rt_cpu").as_int(); // NOLINT
        RCLCPP_INFO(*logging::get_sys_logger(), "Using rt-cpu: %d", rt_cpu_);

        this->declare_parameter<std::string>("non_rt_cpus", "0-5,7-15");
        non_rt_cpus_ = this->get_parameter("non_rt_cpus").as_string();
        RCLCPP_INFO(*logging::get_sys_logger(), "Using non_rt_cpus: %s", non_rt_cpus_.c_str());

        this->declare_parameter<std::string>(
            "config_file", "/home/hang/ecat_ws/src/soem_wrapper/config/config.yaml");
        config_file_ = this->get_parameter("config_file").as_string();
        RCLCPP_INFO(*logging::get_cfg_logger(), "Using config_file: %s", config_file_.c_str());
        get_configuration_data()->load_initial_value_from_config(config_file_);
        RCLCPP_INFO(*logging::get_cfg_logger(), "Configuration file loaded");

        register_components();
    }

    EthercatNode::~EthercatNode() {
        running_ = false;
    }

    void EthercatNode::on_shutdown() {
        RCLCPP_INFO(*logging::get_sys_logger(), "Shutting down");

        exiting_ = true;
        // wait until reset command sent
        while (!exiting_reset_called_) {
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // then wait for another 100ms for slaves to reset actuators
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(*logging::get_data_logger(), "Stop data cycle");
        running_ = false;
        if (data_thread_.joinable()) {
            data_thread_.join();
        }
        if (checker_thread_.joinable()) {
            checker_thread_.join();
        }

        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        RCLCPP_INFO(*logging::get_sys_logger(), "Init state for all slaves requested");
        ec_close();
    }

    void EthercatNode::datacycle_callback() {
        // set soem_wrapper cpu affinity
        const pthread_t thread_id = pthread_self();
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(rt_cpu_, &cpuset);
        int result = pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpuset);
        if (result != 0) {
            RCLCPP_ERROR(*logging::get_sys_logger(), "Failed to set CPU affinity");
        }

        // set thread priority
        // 49 to make it less than the nic irq
        sched_param sch_params{};
        sch_params.sched_priority = 49;
        result = pthread_setschedparam(thread_id, SCHED_FIFO, &sch_params);
        if (result != 0) {
            RCLCPP_ERROR(*logging::get_sys_logger(), "Failed to set thread priority.");
        } else {
            RCLCPP_INFO(*logging::get_sys_logger(), "Thread priority set to 49 with SCHED_FIFO");
        }

        // move other threads in this cpu
        utils::sys::move_threads(rt_cpu_, non_rt_cpus_, interface_);
        RCLCPP_INFO(*logging::get_sys_logger(), "move threads finished");

        // bind nic irq to same cpu core
        utils::sys::move_irq(rt_cpu_, interface_);
        RCLCPP_INFO(*logging::get_sys_logger(), "bind irq finished");

        // optimize nic settings
        utils::sys::setup_nic(interface_);
        RCLCPP_INFO(*logging::get_sys_logger(), "setup nic finished");

        bool all_slave_ready = false;
        rclcpp::Time current_time{};

        // all settings updated, mark data cycle as operational
        in_operational_ = true;

        while (running_) {
            // recv ecat frame
            wkc_ = ec_receive_processdata(100);

            // transfer data from ecat stack into buffer managed by ourselves
            for (const auto &slave: get_slave_devices()) {
                std::lock_guard lock(slave->mtx_);
                slave->receive_from_slave();
            }

            // check if all slaves are all ready
            if (!all_slave_ready) {
                // initially true
                bool all_ready = true;
                // check state of all slaves
                for (const auto &slave: get_slave_devices()) {
                    std::lock_guard lock(slave->mtx_);
                    if (*slave->get_slave_status_ptr()
                        < SLAVE_CONFIRM_READY
                        || !slave->is_ready()) {
                        // any one not ready, mark the final result as not ready
                        all_ready = false;
                        break;
                    }
                }
                // if all ready, log ready
                if (all_ready) {
                    all_slave_ready = true;
                    RCLCPP_INFO(*logging::get_data_logger(),
                                "========== All %d slave(s) ready, system started ==========",
                                ec_slavecount);
                }
            }

            // process pdo device by devices
            for (const auto &slave: get_slave_devices()) {
                std::lock_guard lock(slave->mtx_);

                // if this device is not fully configured, skip pdo processing
                if (slave->is_conf_ros_done() == 0
                    || slave->is_ecat_conf_done() == 0) {
                    continue;
                }

                // slave report that all args are well-received
                if (*slave->get_slave_status_ptr() == SLAVE_CONFIRM_READY
                    && !slave->is_ready()) {
                    RCLCPP_INFO(*logging::get_data_logger(), "Slave id=%d confirmed ready", slave->get_index());
                    slave->set_ready(true);
                }

                // sending args
                // master will send arg bytes one by one
                // slave will send what it receives back to the master
                // to ensure the data is correct
                if (*slave->get_master_status_ptr() == MASTER_SENDING_ARGUMENTS
                    && !slave->is_arg_sent()) {
                    slave->send_arg();
                }

                // if slave not ready before
                // but updated to ready in this cycle
                if (!slave->is_ready()
                    && slave->is_arg_sent()
                    && *slave->get_slave_status_ptr() == SLAVE_READY) {
                    // write initial value for each app
                    // only write in first initialization
                    if (*slave->get_master_status_ptr() != MASTER_READY) {
                        if (slave->get_reconnected_times() == 0) {
                            slave->write_init_values();
                        } else {
                            slave->recover_master_to_slave_buf();
                            RCLCPP_INFO(*logging::get_health_checker_logger(),
                                        "Slave id=%d master to slave buf recovered", slave->get_index());
                        }

                        // after this slave will go into normal working state
                        RCLCPP_INFO(*logging::get_data_logger(),
                                    "Slave id=%d sdo confirmed received", slave->get_index());
                    }

                    *slave->get_master_status_ptr() = MASTER_READY;
                }

                // if slave is ready/working
                if (slave->is_ready()) {
                    current_time = rclcpp::Clock().now();
                    slave->process_pdo(current_time);
                }
            }

            // if configuration is finished and exiting
            // then override all tasks with the init value
            // thereby resetting all the actuators in the slave
            if (exiting_ && !exiting_reset_called_) {
                for (const auto &slave: get_slave_devices()) {
                    if (slave->is_arg_sent()) {
                        slave->write_init_values();
                        RCLCPP_INFO(*logging::get_data_logger(), "Slave id=%d exit reset command sent",
                                    slave->get_index());
                    }
                }
                exiting_reset_called_ = true;
            }

            // transfer pdo data from buffer managed by ourselves info ecat stack
            for (const auto &slave: get_slave_devices()) {
                std::lock_guard lock(slave->mtx_);
                slave->transfer_to_slave();
            }

            // send ecat frame
            ec_send_processdata();
        }

        // destroy all publisher and subscriber
        for (const auto &slave: get_slave_devices()) {
            std::lock_guard lock(slave->mtx_);
            slave->cleanup();
        }
        RCLCPP_INFO(*logging::get_data_logger(), "DATA thread exiting...");
    }

    void EthercatNode::state_check_callback() const {
        // pre-define var outside the loop
        // to save time and improve perf
        int slave_idx{};

        while (running_ && rclcpp::ok()) {
            if (in_operational_ && (wkc_ < expectedWkc_ || ec_group[0].docheckstate)) {
                RCLCPP_WARN_THROTTLE(*logging::get_health_checker_logger(),
                                     *get_clock(),
                                     1500,
                                     "Enter state check, wkc=%d, expected wkc=%d, lastFailed=%d",
                                     wkc_.load(),
                                     expectedWkc_,
                                     ec_group[0].docheckstate);
                ec_group[0].docheckstate = FALSE;
                ec_readstate();

                // ReSharper disable once CppJoinDeclarationAndAssignment
                for (const auto &slave: get_slave_devices()) {
                    slave_idx = slave->get_index();

                    RCLCPP_WARN_THROTTLE(
                        *logging::get_health_checker_logger(),
                        *get_clock(),
                        1500,
                        "Checking slave idx=%d, "
                        "state = %d",
                        slave_idx,
                        ec_slave[slave_idx].state);

                    if (ec_slave[slave_idx].state != EC_STATE_OPERATIONAL) {
                        ec_group[0].docheckstate = TRUE;

                        // reconnected but slave restarted
                        // resend all args
                        if (ec_slave[slave_idx].state == EC_STATE_SAFE_OP
                            && !slave->is_recover_rejected()) {
                            RCLCPP_INFO(*logging::get_health_checker_logger(),
                                        "Slave idx=%d back to safe-op, state to op", slave_idx);
                            {
                                std::lock_guard lock(slave->mtx_);
                                slave->pre_recover();
                            }

                            ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
                            ec_writestate(slave_idx);
                            ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
                            RCLCPP_INFO(*logging::get_health_checker_logger(),
                                        "Slave idx=%d back to op, resending sdo", slave_idx);
                        } else if (ec_slave[slave_idx].state == EC_STATE_INIT) {
                            // double check state
                            if (ec_statecheck(slave_idx, EC_STATE_INIT, 1000)) {
                                if (ec_reconfig_slave(slave_idx, 500)) {
                                    // another slave reconnected at the old position
                                    if (slave->is_recover_rejected()) {
                                        RCLCPP_ERROR_THROTTLE(
                                            *logging::get_health_checker_logger(),
                                            *get_clock(),
                                            1500,
                                            "Slave idx=%d connected with different board, recover rejected",
                                            slave_idx);
                                        ec_slave[slave_idx].state = EC_STATE_INIT;
                                        ec_writestate(slave_idx);
                                    } else {
                                        // same slave reconnected
                                        ec_slave[slave_idx].islost = FALSE;
                                        RCLCPP_INFO(*logging::get_health_checker_logger(),
                                                    "Slave idx=%d reconfigured", slave_idx);
                                    }
                                }

                                std::lock_guard lock(slave->mtx_);
                                slave->reset_state();
                            }
                        } else if (!ec_slave[slave_idx].islost) {
                            if (ec_slave[slave_idx].state == EC_STATE_NONE) {
                                RCLCPP_ERROR(*logging::get_health_checker_logger(), "Slave idx=%d lost", slave_idx);
                                ec_slave[slave_idx].islost = TRUE;

                                std::lock_guard lock(slave->mtx_);
                                slave->reset_state();
                                slave->on_connection_lost();
                                slave->backup_master_to_slave_buf();
                            }
                        }
                    }

                    if (ec_slave[slave_idx].islost) {
                        if (ec_slave[slave_idx].state == EC_STATE_NONE) {
                            if (ec_recover_slave(slave_idx, 500)) {
                                ec_slave[slave_idx].islost = FALSE;
                                RCLCPP_INFO(*logging::get_health_checker_logger(), "Slave idx=%d recovered", slave_idx);
                                slave->recover_state();
                            }
                        } else {
                            ec_slave[slave_idx].islost = FALSE;

                            std::lock_guard lock(slave->mtx_);
                            slave->recover_state();
                            RCLCPP_INFO(*logging::get_health_checker_logger(), "Slave idx=%d found", slave_idx);
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool EthercatNode::setup_ecat() {
        if (!ec_init(interface_.c_str())) {
            RCLCPP_ERROR(*logging::get_sys_logger(), "No socket connection on %s. \n", interface_.c_str());
            return false;
        }
        RCLCPP_INFO(*logging::get_sys_logger(), "ec_init on %s succeeded.", interface_.c_str());

        if (ec_config_init(FALSE) <= 0) {
            RCLCPP_ERROR(*logging::get_cfg_logger(), "No slaves found!");
            return false;
        }

        RCLCPP_INFO(*logging::get_cfg_logger(), "%d slaves found", ec_slavecount);
        init_slave_devices_vector(ec_slavecount);
        // write back to init state
        for (int i = 1; i <= ec_slavecount; i++) {
            ec_slave[i].state = EC_STATE_INIT;
            ec_writestate(i);
        }
        ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);

        RCLCPP_INFO(*logging::get_cfg_logger(), "all slaves backed to init, restarting mapping");
        const int reconf_slaves = ec_config_init(FALSE);
        RCLCPP_INFO(*logging::get_cfg_logger(), "detected %d slaves", reconf_slaves);
        // setup conf func
        for (int i = 1; i <= ec_slavecount; i++) {
            ec_slave[i].PO2SOconfigx = config_ec_slave;
        }

        // conf io map
        ec_config_map(&IOmap_);
        ec_configdc();

        for (const auto &device: get_slave_devices()) {
            if (!device->init_task_list()) {
                return false;
            }
        }

        // change slave state to op
        RCLCPP_INFO(*logging::get_cfg_logger(), "Slaves mapped, state to SAFE_OP.");
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

        RCLCPP_INFO(*logging::get_cfg_logger(), "All slaves reached SAFE_OP, state to OP");
        expectedWkc_ = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;

        RCLCPP_INFO(*logging::get_cfg_logger(), "Calculated expected wkc = %d", expectedWkc_);
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_writestate(0);

        // mock data process
        int chk = 50;
        do {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);
        } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

        // pre-final-op state check
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
            RCLCPP_INFO(*logging::get_cfg_logger(), "Operational state reached for all slaves.");
            data_thread_ = std::thread(&EthercatNode::datacycle_callback, this);
            checker_thread_ = std::thread(&EthercatNode::state_check_callback, this);
        }

        return true;
    }

    void EthercatNode::register_components() {
        // deprecated
        // register_module(1, "FlightModule", 16, 40, 001);
        // register_module(2, "MotorModule", 56, 80, 001);
        register_module(3, "H750UniversalModule", 80, 80, 006);
    }

    void EthercatNode::register_module(const uint32_t eep_id,
                                       const std::string &module_name,
                                       const int master_to_slave_buf_len,
                                       const int slave_to_master_buf_len,
                                       const int min_sw_rev) {
        RCLCPP_INFO(*logging::get_wrapper_logger(),
                    "Registered new module, eepid=%d, name=%s, m2slen=%d, s2mlen=%d",
                    eep_id,
                    module_name.c_str(),
                    master_to_slave_buf_len,
                    slave_to_master_buf_len);
        registered_module_names[eep_id] = module_name;
        registered_module_buf_lens[eep_id] =
                std::make_pair(master_to_slave_buf_len, slave_to_master_buf_len);
        registered_module_sw_rev[eep_id] = min_sw_rev;
    }
}
