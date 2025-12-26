//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/defs/task_defs.hpp"
#include "soem_wrapper/defs/msg_defs.hpp"
#include "soem_wrapper/utils/sys_utils.hpp"

namespace aim::ecat {
    std::shared_ptr<EthercatNode> node = nullptr;

    void register_node() {
        node = std::make_shared<EthercatNode>();
    }

    std::shared_ptr<EthercatNode> get_node() {
        return node;
    }

    void destroy_node() {
        node.reset();
    }

    EthercatNode::EthercatNode() : Node("EthercatNode"), running_(true) {
        this->declare_parameter<std::string>("interface", "enp2s0");
        interface = this->get_parameter("interface").as_string();
        RCLCPP_INFO(*logging::get_sys_logger(), "Using interface: %s", interface.c_str());

        this->declare_parameter<int>("rt_cpu", 6);
        rt_cpu = this->get_parameter("rt_cpu").as_int(); // NOLINT
        RCLCPP_INFO(*logging::get_sys_logger(), "Using rt-cpu: %d", rt_cpu);

        this->declare_parameter<std::string>("non_rt_cpus", "0-5,7-15");
        non_rt_cpus = this->get_parameter("non_rt_cpus").as_string();
        RCLCPP_INFO(*logging::get_sys_logger(), "Using non_rt_cpus: %s", non_rt_cpus.c_str());

        this->declare_parameter<std::string>(
            "config_file", "/home/hang/ecat_ws/src/soem_wrapper/config/config.yaml");
        config_file = this->get_parameter("config_file").as_string();
        RCLCPP_INFO(*logging::get_cfg_logger(), "Using config_file: %s", config_file.c_str());
        get_dynamic_data()->load_initial_value_from_config(config_file);
        RCLCPP_INFO(*logging::get_cfg_logger(), "Configuration file loaded");

        register_components();
    }

    EthercatNode::~EthercatNode() {
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
        CPU_SET(rt_cpu, &cpuset);
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
        utils::sys::move_threads(rt_cpu, non_rt_cpus, interface);
        RCLCPP_INFO(*logging::get_sys_logger(), "move threads finished");

        // bind nic irq to same cpu core
        utils::sys::move_irq(rt_cpu, interface);
        RCLCPP_INFO(*logging::get_sys_logger(), "bind irq finished");

        // optimize nic settings
        utils::sys::setup_nic(interface);
        RCLCPP_INFO(*logging::get_sys_logger(), "setup nic finished");

        // pre-define var outside the loop
        // to save time and improve perf
        int slave_idx{};
        int app_idx{};
        uint8_t task_type{};
        uint16_t pdo_offset{};
        int offset{};
        bool all_slave_ready = false;
        rclcpp::Time current_time{};
        std_msgs::msg::Float32 std_msgs_float32_shared_msg{};

        // all settings updated, mark data cycle as operational
        in_operational_ = true;

        while (running_ && rclcpp::ok()) {
            // recv ecat frame
            wkc = ec_receive_processdata(100);

            // transfer data from ecat stack into buffer managed by ourselves
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                memcpy(&get_slave_devices()[slave_idx].slave_status, ec_slave[slave_idx].inputs, 1);
                memcpy(get_slave_devices()[slave_idx].slave_to_master_buf.data(), ec_slave[slave_idx].inputs + 1,
                       get_slave_devices()[slave_idx].slave_to_master_buf_len);
            }

            // check if all slaves are all ready
            if (!all_slave_ready) {
                // initially true
                bool all_ready = true;
                // check state of all slaves
                for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
                    std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                    if (get_slave_devices()[slave_idx].slave_status < SLAVE_CONFIRM_READY
                        || get_slave_devices()[slave_idx].ready == 0) {
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
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);

                // if this device is not fully configured, skip pdo processing
                if (get_slave_devices()[slave_idx].conf_ros_done == 0
                    || get_slave_devices()[slave_idx].conf_done == 0) {
                    continue;
                }

                // slave report that all args are well-received
                if (get_slave_devices()[slave_idx].slave_status == SLAVE_CONFIRM_READY
                    && get_slave_devices()[slave_idx].ready == 0) {
                    RCLCPP_INFO(*logging::get_data_logger(), "Slave id=%d confirmed ready", slave_idx);
                    get_slave_devices()[slave_idx].ready = 1;
                }

                // sending args
                // master will send arg bytes one by one
                // slave will send what it receives back to the master
                // to ensure the data is correct
                if (get_slave_devices()[slave_idx].master_status == MASTER_SENDING_ARGUMENTS
                    && get_slave_devices()[slave_idx].arg_sent == 0) {
                    // send-back arg byte from slave
                    const uint8_t arg_recv = get_slave_devices()[slave_idx].slave_to_master_buf[2];
                    const uint16_t arg_recv_idx =
                            static_cast<uint16_t>(get_slave_devices()[slave_idx].slave_to_master_buf[1]) << 8
                            | get_slave_devices()[slave_idx].slave_to_master_buf[0];

                    // all arg bytes are confirmed
                    if (arg_recv_idx == get_slave_devices()[slave_idx].sending_arg_buf_idx
                        && arg_recv == get_slave_devices()[slave_idx].arg_buf
                        [get_slave_devices()[slave_idx].sending_arg_buf_idx - 1]) {
                        if (get_slave_devices()[slave_idx].sending_arg_buf_idx == get_slave_devices()[slave_idx].
                            arg_buf_len) {
                            RCLCPP_INFO(*logging::get_data_logger(), "Slave id=%d sdo all sent", slave_idx);
                            get_slave_devices()[slave_idx].arg_sent = 1;
                        } else {
                            get_slave_devices()[slave_idx].sending_arg_buf_idx++;
                        }
                    }

                    // sending arg bytes
                    if (get_slave_devices()[slave_idx].arg_sent == 0) {
                        RCLCPP_DEBUG(*logging::get_data_logger(),
                                     "Slave id=%d sending sdo idx %d / %d, content= %d, "
                                     "slavestatus=%d, masterstatus=%d",
                                     slave_idx, get_slave_devices()[slave_idx].sending_arg_buf_idx,
                                     get_slave_devices()[slave_idx].arg_buf_len,
                                     get_slave_devices()[slave_idx].arg_buf[get_slave_devices()[slave_idx].
                                         sending_arg_buf_idx - 1],
                                     get_slave_devices()[slave_idx].slave_status,
                                     get_slave_devices()[slave_idx].master_status);
                        get_slave_devices()[slave_idx].master_to_slave_buf[0] =
                                get_slave_devices()[slave_idx].sending_arg_buf_idx & 0xFF;
                        get_slave_devices()[slave_idx].master_to_slave_buf[1] =
                                get_slave_devices()[slave_idx].sending_arg_buf_idx >> 8 & 0xFF;
                        get_slave_devices()[slave_idx].master_to_slave_buf[2] =
                                get_slave_devices()[slave_idx].arg_buf[
                                    get_slave_devices()[slave_idx].sending_arg_buf_idx - 1];
                    }
                }

                // if slave not ready before
                // but updated to ready in this cycle
                if (get_slave_devices()[slave_idx].ready == 0
                    && get_slave_devices()[slave_idx].arg_sent == 1
                    && get_slave_devices()[slave_idx].slave_status == SLAVE_READY) {
                    // write initial value for each app
                    // only write in first initialization
                    if (get_slave_devices()[slave_idx].reconnected_times == 0
                        && get_slave_devices()[slave_idx].master_status != MASTER_READY) {
                        memset(get_slave_devices()[slave_idx].master_to_slave_buf.data(),
                               0, get_slave_devices()[slave_idx].master_to_slave_buf_len);

                        // write init value
                        for (app_idx = 1;
                             app_idx <= get_field_as<uint8_t>(*get_dynamic_data(),
                                                              get_slave_devices()[slave_idx].sn,
                                                              "task_count");
                             app_idx++) {
                            // if a task dont have a subscriber
                            // it means its not a controllable task
                            // no need to write any data
                            if (!get_dynamic_data()->has(
                                    fmt::format("sn{}_app_{}_sub_inst",
                                                get_slave_devices()[slave_idx].sn,
                                                app_idx))
                            ) {
                                continue;
                            }

                            // define outside loop to improve perf
                            // ReSharper disable once CppJoinDeclarationAndAssignment
                            task_type = get_field_as<uint8_t>(
                                *get_dynamic_data(),
                                get_slave_devices()[slave_idx].sn,
                                app_idx,
                                "sdowrite_task_type");
                            // ReSharper disable once CppJoinDeclarationAndAssignment
                            pdo_offset = get_field_as<uint16_t>(
                                *get_dynamic_data(),
                                get_slave_devices()[slave_idx].sn,
                                app_idx,
                                "pdowrite_offset");
                            offset = pdo_offset;

                            app_registry.at(task_type)->init_value(
                                get_slave_devices()[slave_idx].master_to_slave_buf.data(),
                                &offset,
                                fmt::format("sn{}_app_{}_", get_slave_devices()[slave_idx].sn, app_idx)
                            );
                        }

                        // after this slave will go into normal working state
                        RCLCPP_INFO(*logging::get_data_logger(), "slave id %d sdo confirmed received", slave_idx);
                    }

                    get_slave_devices()[slave_idx].master_status = MASTER_READY;
                }

                // if slave is ready/working
                if (get_slave_devices()[slave_idx].ready == 1) {
                    // latency calc
                    // if the slaves works well
                    // master will continuous sending an incremental number in get_slave_devices()[slave_idx].master_status
                    // and slave will send it back in get_slave_devices()[slave_idx].slave_status
                    // the travel latency is the recv time - sent time
                    if (get_slave_devices()[slave_idx].waiting_for_latency_checking == 1) {
                        if (get_slave_devices()[slave_idx].slave_status == get_slave_devices()[slave_idx].
                            master_status) {
                            current_time = rclcpp::Clock().now();
                            // unit: ms
                            std_msgs_float32_shared_msg.data =
                                    static_cast<float>(current_time.seconds()
                                                       - get_slave_devices()[slave_idx].last_packet_time.seconds())
                                    * 1000.f;
                            get_slave_devices()[slave_idx].last_packet_time = current_time;
                            get_slave_devices()[slave_idx].waiting_for_latency_checking = 0;

                            // publish latency data
                            publish_msg<std_msgs::msg::Float32>(
                                fmt::format("sn{}_latency_", get_slave_devices()[slave_idx].sn),
                                std_msgs_float32_shared_msg);

                            // data processing
                            for (app_idx = 1;
                                 app_idx <= get_field_as<uint8_t>(
                                     *get_dynamic_data(),
                                     get_slave_devices()[slave_idx].sn,
                                     "task_count");
                                 app_idx++) {
                                // if a task dont have a publisher
                                // it means its not a reportable task
                                // no need to read any data
                                if (!get_dynamic_data()->has(
                                    fmt::format("sn{}_app_{}_pub_inst",
                                                get_slave_devices()[slave_idx].sn,
                                                app_idx))) {
                                    continue;
                                }

                                // define outside loop to improve perf
                                // ReSharper disable once CppJoinDeclarationAndAssignment
                                task_type = get_field_as<uint8_t>(
                                    *get_dynamic_data(),
                                    get_slave_devices()[slave_idx].sn,
                                    app_idx,
                                    "sdowrite_task_type");
                                // ReSharper disable once CppJoinDeclarationAndAssignment
                                pdo_offset = get_field_as<uint16_t>(
                                    *get_dynamic_data(),
                                    get_slave_devices()[slave_idx].sn,
                                    app_idx,
                                    "pdoread_offset");
                                offset = pdo_offset;

                                app_registry.at(task_type)->read(
                                    get_slave_devices()[slave_idx].slave_to_master_buf.data(),
                                    &offset,
                                    fmt::format("sn{}_app_{}_", get_slave_devices()[slave_idx].sn, app_idx)
                                );
                            }
                        }
                    } else {
                        // latency calculation number increments here
                        if (get_slave_devices()[slave_idx].master_status >= 250) {
                            get_slave_devices()[slave_idx].master_status = MASTER_READY + 2;
                        } else {
                            get_slave_devices()[slave_idx].master_status++;
                        }
                        get_slave_devices()[slave_idx].waiting_for_latency_checking = 1;
                    }
                }
            }

            // transfer pdo data from buffer managed by ourselves info ecat stack
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                memcpy(ec_slave[slave_idx].outputs, &get_slave_devices()[slave_idx].master_status, 1);
                memcpy(ec_slave[slave_idx].outputs + 1, get_slave_devices()[slave_idx].master_to_slave_buf.data(),
                       get_slave_devices()[slave_idx].master_to_slave_buf_len);
            }

            // send ecat frame
            ec_send_processdata();
        }

        // destroy all publisher and subscriber
        get_dynamic_data()->reset_and_remove_publishers();
        RCLCPP_INFO(*logging::get_data_logger(), "DATA thread exiting...");
    }

    void EthercatNode::state_check_callback() const {
        // pre-define var outside the loop
        // to save time and improve perf
        int slave_idx{};
        int app_idx{};
        uint8_t task_type{};
        uint16_t pdo_offset{};
        int offset{};

        while (running_ && rclcpp::ok()) {
            if (in_operational_ && (wkc < expectedWkc || ec_group[0].docheckstate)) {
                RCLCPP_WARN_THROTTLE(*logging::get_health_checker_logger(),
                                     *get_clock(),
                                     1500,
                                     "Enter state check, wkc=%d, expected wkc=%d, lastFailed=%d",
                                     wkc.load(),
                                     expectedWkc,
                                     ec_group[0].docheckstate);
                ec_group[0].docheckstate = FALSE;
                ec_readstate();

                // ReSharper disable once CppJoinDeclarationAndAssignment
                for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
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
                            && !get_slave_devices()[slave_idx].recover_rejected) {
                            RCLCPP_INFO(*logging::get_health_checker_logger(),
                                        "Slave idx=%d back to safe-op, state to op", slave_idx);
                            {
                                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                                get_slave_devices()[slave_idx].arg_sent = 0;
                                get_slave_devices()[slave_idx].sending_arg_buf_idx = 1;
                                get_slave_devices()[slave_idx].reconnected_times++;
                                get_slave_devices()[slave_idx].conf_ros_done = 1;
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
                                    if (get_slave_devices()[slave_idx].recover_rejected) {
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

                                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                                get_slave_devices()[slave_idx].ready = 0;
                                get_slave_devices()[slave_idx].conf_ros_done = 0;
                                get_slave_devices()[slave_idx].slave_status = SLAVE_INITIALIZING;
                            }
                        } else if (!ec_slave[slave_idx].islost) {
                            if (ec_slave[slave_idx].state == EC_STATE_NONE) {
                                RCLCPP_ERROR(*logging::get_health_checker_logger(), "Slave idx=%d lost", slave_idx);
                                ec_slave[slave_idx].islost = TRUE;

                                std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                                get_slave_devices()[slave_idx].ready = 1;
                                get_slave_devices()[slave_idx].conf_ros_done = 1;
                                // ReSharper disable once CppJoinDeclarationAndAssignment
                                for (app_idx = 1;
                                     app_idx <= get_field_as<uint8_t>(
                                         *get_dynamic_data(),
                                         get_slave_devices()[slave_idx].sn,
                                         "task_count");
                                     app_idx++) {
                                    // process mst to slv connection lost actions
                                    // 0x02 = Reset to default
                                    if (get_field_as<uint8_t>(
                                            *get_dynamic_data(),
                                            get_slave_devices()[slave_idx].sn,
                                            app_idx,
                                            "sdowrite_connection_lost_write_action",
                                            0) == 0x02) {
                                        // ReSharper disable once CppJoinDeclarationAndAssignment
                                        task_type = get_field_as<uint8_t>(
                                            *get_dynamic_data(),
                                            get_slave_devices()[slave_idx].sn,
                                            app_idx,
                                            "sdowrite_task_type");
                                        // ReSharper disable once CppJoinDeclarationAndAssignment
                                        pdo_offset = get_field_as<uint16_t>(
                                            *get_dynamic_data(),
                                            get_slave_devices()[slave_idx].sn,
                                            app_idx,
                                            "pdowrite_offset");
                                        offset = pdo_offset;

                                        app_registry.at(task_type)->init_value(
                                            get_slave_devices()[slave_idx].master_to_slave_buf.data(),
                                            &offset,
                                            fmt::format("sn{}_app_{}_",
                                                        get_slave_devices()[slave_idx].sn,
                                                        app_idx)
                                        );

                                        RCLCPP_WARN(*logging::get_health_checker_logger(),
                                                    "Slave idx=%d, task id=%d, control command has been reset",
                                                    slave_idx,
                                                    app_idx);
                                    }

                                    // process slv to mst connection lost actions
                                    if (get_field_as<uint8_t>(
                                            *get_dynamic_data(),
                                            get_slave_devices()[slave_idx].sn,
                                            app_idx,
                                            "conf_connection_lost_read_action",
                                            0) == 0x02) {
                                        app_registry.at(task_type)->publish_empty_message(
                                            fmt::format("sn{}_app_{}_",
                                                        get_slave_devices()[slave_idx].sn,
                                                        app_idx)
                                        );

                                        RCLCPP_WARN(*logging::get_health_checker_logger(),
                                                    "Slave idx=%d, task id=%d, report message has been reset",
                                                    slave_idx,
                                                    app_idx);
                                    }
                                }
                            }
                        }
                    }

                    if (ec_slave[slave_idx].islost) {
                        if (ec_slave[slave_idx].state == EC_STATE_NONE) {
                            if (ec_recover_slave(slave_idx, 500)) {
                                ec_slave[slave_idx].islost = FALSE;
                                RCLCPP_INFO(*logging::get_health_checker_logger(), "Slave idx=%d recovered", slave_idx);
                            }
                        } else {
                            ec_slave[slave_idx].islost = FALSE;

                            std::lock_guard lock(get_slave_devices()[slave_idx].mtx);
                            get_slave_devices()[slave_idx].ready = 1;
                            get_slave_devices()[slave_idx].conf_ros_done = 1;
                            RCLCPP_INFO(*logging::get_health_checker_logger(), "Slave idx=%d found", slave_idx);
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool EthercatNode::setup_ecat() {
        if (!ec_init(interface.c_str())) {
            RCLCPP_ERROR(*logging::get_sys_logger(), "No socket connection on %s. \n", interface.c_str());
            return false;
        }
        RCLCPP_INFO(*logging::get_sys_logger(), "ec_init on %s succeeded.", interface.c_str());

        if (ec_config_init(FALSE) <= 0) {
            RCLCPP_ERROR(*logging::get_cfg_logger(), "No slaves found!");
            return false;
        }

        RCLCPP_INFO(*logging::get_cfg_logger(), "%d slaves found", ec_slavecount);
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
            std::lock_guard lock(get_slave_devices()[i].mtx);
            get_slave_devices()[i].slave = &ec_slave[i];
        }

        // conf io map
        ec_config_map(&IOmap);
        ec_configdc();

        for (int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
            std::lock_guard lock(get_slave_devices()[slave_idx].mtx);

            if (get_slave_devices()[slave_idx].sw_rev_check_passed == 0) {
                return false;
            }

            // create arg buffer
            get_slave_devices()[slave_idx].arg_buf_len = get_field_as<uint16_t>(
                *get_dynamic_data(),
                get_slave_devices()[slave_idx].sn,
                "sdo_len");
            get_slave_devices()[slave_idx].arg_buf.resize(get_slave_devices()[slave_idx].arg_buf_len);
            get_slave_devices()[slave_idx].arg_buf.clear();
            get_slave_devices()[slave_idx].arg_buf_idx = 0;

            // create latency publisher for each module
            create_and_insert_publisher<std_msgs::msg::Float32>(
                fmt::format("sn{}_latency_", get_slave_devices()[slave_idx].sn));

            // write basic args
            const uint8_t task_count = get_field_as<uint8_t>(
                *get_dynamic_data(),
                get_slave_devices()[slave_idx].sn,
                "task_count"); // NOLINT(*-use-auto)

            if (task_count == 0) {
                RCLCPP_ERROR(*logging::get_cfg_logger(),
                             "Slave %d don't have any task, please add at least one or remove this slave",
                             slave_idx);
                return false;
            }
            get_slave_devices()[slave_idx].arg_buf[get_slave_devices()[slave_idx].arg_buf_idx++] = task_count;

            for (int app_idx = 1; app_idx <= task_count; app_idx++) {
                const uint8_t task_type = get_field_as<uint8_t>(
                    *get_dynamic_data(),
                    get_slave_devices()[slave_idx].sn,
                    app_idx,
                    "sdowrite_task_type"); // NOLINT(*-use-auto)
                // write basic args
                auto [sdo_buf, sdo_len] = get_dynamic_data()->build_buf(
                    fmt::format("sn{}_app_{}_sdowrite_",
                                get_slave_devices()[slave_idx].sn,
                                app_idx),
                    {
                        "task_type"
                    }
                );
                memcpy(get_slave_devices()[slave_idx].arg_buf.data()
                       + get_slave_devices()[slave_idx].arg_buf_idx,
                       sdo_buf,
                       sdo_len);
                get_slave_devices()[slave_idx].arg_buf_idx++;

                // call init func of each app/task
                app_registry.at(task_type)->init_sdo(
                    get_slave_devices()[slave_idx].arg_buf.data(),
                    &get_slave_devices()[slave_idx].arg_buf_idx,
                    get_slave_devices()[slave_idx].sn,
                    slave_idx,
                    fmt::format("sn{}_app_{}_", get_slave_devices()[slave_idx].sn, app_idx)
                );
            }

            // mark ros conf step as done
            get_slave_devices()[slave_idx].conf_ros_done = 1;
        }

        // change slave state to op
        RCLCPP_INFO(*logging::get_cfg_logger(), "Slaves mapped, state to SAFE_OP.");
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

        RCLCPP_INFO(*logging::get_cfg_logger(), "All slaves reached SAFE_OP, state to OP");
        expectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;

        RCLCPP_INFO(*logging::get_cfg_logger(), "Calculated expected wkc = %d", expectedWkc);
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
        register_module(3, "H750UniversalModule", 80, 80, 005);

        register_app(&task::dbus_rc::DBUS_RC::instance());
        register_app(&task::hipnuc_imu::HIPNUC_IMU_CAN::instance());
        register_app(&task::dji_motor::DJI_MOTOR::instance());
        register_app(&task::dshot::DSHOT::instance());
        register_app(&task::pwm::ONBOARD_PWM::instance());
        // register_app<EXTERNAL_PWM>();
        // register_app<MS5837_30BA>();
        register_app(&task::lk_motor::LK_MOTOR::instance());
        // register_app<ADC>();
        // register_app<CAN_PMU>();
        register_app(&task::sbus_rc::SBUS_RC::instance());
        register_app(&task::dm_motor::DM_MOTOR::instance());
    }

    void EthercatNode::register_module(const uint32_t eep_id, const std::string &module_name,
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

    void EthercatNode::register_app(task::TaskWrapper *task_wrapper) {
        RCLCPP_INFO(*logging::get_wrapper_logger(),
                    "Registered new app, id=%d, name=%s",
                    task_wrapper->get_type_id(),
                    task_wrapper->get_type_name().c_str());
        app_registry[task_wrapper->get_type_id()] = task_wrapper;
    }

    template<typename MsgT>
    void EthercatNode::create_and_insert_publisher(const std::string &prefix) {
        get_dynamic_data()->set(
            fmt::format("{}pub_inst", prefix),
            this->create_publisher<MsgT>(
                get_field_as<std::string>(
                    *get_dynamic_data(),
                    fmt::format("{}pub_topic", prefix)),
                rclcpp::SensorDataQoS()
            )
        );
        RCLCPP_DEBUG(*logging::get_cfg_logger(),
                     "Publisher created key=%spub_inst", prefix.c_str());
    }

    template<typename MsgT>
    void EthercatNode::publish_msg(const std::string &prefix, const MsgT &msg) {
        std::get<typename rclcpp::Publisher<MsgT>::SharedPtr>(
            get_dynamic_data()->get(
                fmt::format("{}pub_inst", prefix)
            ))->publish(msg);
    }

    template<typename MsgT>
    void EthercatNode::create_and_insert_subscriber(const std::string &prefix, const uint8_t slave_id) {
        get_dynamic_data()->set(
            fmt::format("{}sub_inst", prefix),
            this->create_subscription<MsgT>(
                get_field_as<std::string>(
                    *get_dynamic_data(),
                    fmt::format("{}sub_topic", prefix)),
                rclcpp::SensorDataQoS(),
                [=, this](const typename MsgT::SharedPtr msg) {
                    generic_callback<MsgT>(msg, prefix, slave_id);
                }
            )
        );
        RCLCPP_DEBUG(*logging::get_cfg_logger(), "Subscriber created key=%ssub_inst", prefix.c_str());
    }

    template<typename MsgT>
    void EthercatNode::generic_callback(const typename MsgT::SharedPtr &msg, const std::string &prefix,
                                        const uint8_t slave_id) {
        int pdo_offset_for_cb = get_field_as<uint16_t>(
            *get_dynamic_data(),
            fmt::format("{}pdowrite_offset", prefix));

        std::lock_guard lock(get_slave_devices()[slave_id].mtx);
        task::MsgDef<MsgT>::write(msg, get_slave_devices()[slave_id].master_to_slave_buf.data(), &pdo_offset_for_cb,
                                  prefix);
    }

    template void EthercatNode::create_and_insert_publisher<std_msgs::msg::Float32>(const std::string &);

    template void EthercatNode::publish_msg<
        std_msgs::msg::Float32>(const std::string &, const std_msgs::msg::Float32 &);

    template void EthercatNode::create_and_insert_publisher<custom_msgs::msg::ReadDJIMotor>(const std::string &);

    template void EthercatNode::publish_msg<custom_msgs::msg::ReadDJIMotor>(
        const std::string &, const custom_msgs::msg::ReadDJIMotor &);

    template void EthercatNode::create_and_insert_publisher<custom_msgs::msg::ReadDJIRC>(const std::string &);

    template void EthercatNode::publish_msg<custom_msgs::msg::ReadDJIRC>(
        const std::string &, const custom_msgs::msg::ReadDJIRC &);

    template void EthercatNode::create_and_insert_publisher<custom_msgs::msg::ReadDmMotor>(const std::string &);

    template void EthercatNode::publish_msg<custom_msgs::msg::ReadDmMotor>(
        const std::string &, const custom_msgs::msg::ReadDmMotor &);

    template void EthercatNode::create_and_insert_publisher<custom_msgs::msg::ReadLkMotor>(const std::string &);

    template void EthercatNode::publish_msg<custom_msgs::msg::ReadLkMotor>(
        const std::string &, const custom_msgs::msg::ReadLkMotor &);

    template void EthercatNode::create_and_insert_publisher<custom_msgs::msg::ReadSBUSRC>(const std::string &);

    template void EthercatNode::publish_msg<custom_msgs::msg::ReadSBUSRC>(
        const std::string &, const custom_msgs::msg::ReadSBUSRC &);

    template void EthercatNode::create_and_insert_publisher<sensor_msgs::msg::Imu>(const std::string &);

    template void EthercatNode::publish_msg<sensor_msgs::msg::Imu>(const std::string &, const sensor_msgs::msg::Imu &);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteDJIMotor>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteDJIMotor>(
        const custom_msgs::msg::WriteDJIMotor::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorMITControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteDmMotorMITControl>(
        const custom_msgs::msg::WriteDmMotorMITControl::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<
        custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
        const custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit::SharedPtr &msg, const std::string &prefix,
        uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteDmMotorSpeedControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteDmMotorSpeedControl>(
        const custom_msgs::msg::WriteDmMotorSpeedControl::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<
        custom_msgs::msg::WriteDSHOT>(const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteDSHOT>(
        const custom_msgs::msg::WriteDSHOT::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorOpenloopControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorOpenloopControl>(
        const custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr &msg, const std::string &prefix,
        uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorTorqueControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorTorqueControl>(
        const custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
        const custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr &msg, const std::string &prefix,
        uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>(
        const custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr &msg, const std::string &prefix,
        uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>(
        const custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr &msg, const std::string &prefix,
        uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<
        custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(
        const custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr &msg,
        const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<
        custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>(const std::string &, uint8_t);

    template void EthercatNode::generic_callback<
        custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>(
        const custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr &msg,
        const std::string &prefix, uint8_t slave_id);

    template void EthercatNode::create_and_insert_subscriber<custom_msgs::msg::WriteOnBoardPWM>(
        const std::string &, uint8_t);

    template void EthercatNode::generic_callback<custom_msgs::msg::WriteOnBoardPWM>(
        const custom_msgs::msg::WriteOnBoardPWM::SharedPtr &msg, const std::string &prefix, uint8_t slave_id);
}
