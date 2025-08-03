//
// Created by hang on 25-4-21.
//
#include "soem_wrapper/dynamic_data_helper.hpp"
#include "soem_wrapper/utils.hpp"
#include "soem_wrapper/wrapper.hpp"

#define EC_TIMEOUTMON 500

std::shared_ptr<EthercatNode> node;
rclcpp::Logger sys_logger = rclcpp::get_logger("EthercatNode_SYS");
rclcpp::Logger cfg_logger = rclcpp::get_logger("EthercatNode_CFG");
rclcpp::Logger data_logger = rclcpp::get_logger("EthercatNode_DATA");
rclcpp::Logger wrapper_logger = rclcpp::get_logger("EthercatNode_WRAPPER");

slave_device slave_devices[EC_MAXSLAVE];
int sdo_size_ptr = 4;
uint16_t sdo_size_write_ptr = 0;
rclcpp::Time time_curr;

// pre-create message instant
std_msgs::msg::Float32 std_msgs_float32_shared_msg;
sensor_msgs::msg::Imu sensor_msgs_imu_shared_msg;
custom_msgs::msg::ReadDJIRC custom_msgs_readdjirc_shared_msg;
custom_msgs::msg::ReadDJICAN custom_msgs_readdjican_shared_msg;
custom_msgs::msg::ReadLkMotor custom_msgs_readlkmotor_shared_msg;
custom_msgs::msg::ReadMS5876BA30 custom_msgs_readms5876ba30_shared_msg;
custom_msgs::msg::ReadADC custom_msgs_readadc_shared_msg;
custom_msgs::msg::ReadCANPMU custom_msgs_readcanpmu_shared_msg;
custom_msgs::msg::ReadSBUSRC custom_msgs_readsbusrc_shared_msg;
custom_msgs::msg::ReadDmMotor custom_msgs_readdmmotor_shared_msg;

void EthercatNode::datacycle_callback()
{
    // set soem_wrapper cpu affinity
    const pthread_t thread_id = pthread_self();

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(rt_cpu, &cpuset);

    int result = pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpuset);
    if (result != 0)
    {
        RCLCPP_ERROR(sys_logger, "Failed to set CPU affinity");
    }

    sched_param sch_params{};
    sch_params.sched_priority = 49;
    result = pthread_setschedparam(thread_id, SCHED_FIFO, &sch_params);
    if (result != 0)
    {
        RCLCPP_ERROR(sys_logger, "Failed to set thread priority.");
    }
    else
    {
        RCLCPP_INFO(sys_logger, "Thread priority set to 49 with SCHED_FIFO");
    }

    // move other threads in this cpu
    move_threads(rt_cpu, non_rt_cpus, interface);
    move_irq(rt_cpu, interface);
    setup_nic(interface);

    int slave_idx;
    int app_idx;

    uint8_t task_type;
    uint16_t pdo_offset;
    int offset;

    while (running_ && rclcpp::ok())
    {
        if (pdo_transfer_active)
        {
            // recv ecat frame
            ec_receive_processdata(100);
            // transfer data from ecat stack into buffer managed by ourselves
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
            {
                memcpy(&slave_devices[slave_idx].slave_status, ec_slave[slave_idx].inputs, 1);
                memcpy(slave_devices[slave_idx].slave_to_master_buf.data(), ec_slave[slave_idx].inputs + 1,
                       slave_devices[slave_idx].slave_to_master_buf_len);
            }

            // process pdo device by device
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
            {
                // if this device is not fully configured, skip pdo processing
                if (slave_devices[slave_idx].conf_ros_done == 0 || slave_devices[slave_idx].conf_done == 0)
                {
                    continue;
                }

                // slave report that all args are well-received
                if (slave_devices[slave_idx].slave_status == SLAVE_CONFIRM_READY && slave_devices[slave_idx].ready ==
                    0)
                {
                    RCLCPP_INFO(data_logger, "slave id %d confirmed ready", slave_idx);
                    slave_devices[slave_idx].ready = 1;
                }

                // sending args
                // master will send arg bytes one by one
                // slave will send what it receives back to the master
                // to ensure the data is correct
                if (slave_devices[slave_idx].master_status == MASTER_SENDING_ARGUMENTS && slave_devices[slave_idx].
                    arg_sent == 0)
                {
                    // send-back arg byte from slave
                    const uint8_t arg_recv = slave_devices[slave_idx].slave_to_master_buf[2];
                    const uint16_t arg_recv_idx = static_cast<uint16_t>(slave_devices[slave_idx].slave_to_master_buf[1])
                        << 8 | slave_devices[slave_idx].slave_to_master_buf[0];

                    // all arg bytes are confirmed
                    if (arg_recv_idx == slave_devices[slave_idx].sending_arg_buf_idx &&
                        arg_recv == slave_devices[slave_idx].arg_buf[slave_devices[slave_idx].sending_arg_buf_idx -
                            1])
                    {
                        if (slave_devices[slave_idx].sending_arg_buf_idx == slave_devices[slave_idx].arg_buf_len)
                        {
                            RCLCPP_INFO(data_logger, "slave id %d sdo all sent", slave_idx);
                            slave_devices[slave_idx].arg_sent = 1;
                        }
                        else
                        {
                            slave_devices[slave_idx].sending_arg_buf_idx++;
                        }
                    }

                    // sending arg bytes
                    if (slave_devices[slave_idx].arg_sent == 0)
                    {
                        RCLCPP_INFO(data_logger,
                                    "slave id %d sending sdo idx %d / %d, content= %d, "
                                    "slavestatus=%d, masterstatus=%d",
                                    slave_idx, slave_devices[slave_idx].sending_arg_buf_idx,
                                    slave_devices[slave_idx].arg_buf_len,
                                    slave_devices[slave_idx].arg_buf[slave_devices[slave_idx].sending_arg_buf_idx - 1],
                                    slave_devices[slave_idx].slave_status, slave_devices[slave_idx].master_status);
                        slave_devices[slave_idx].master_to_slave_buf[0] = slave_devices[slave_idx].sending_arg_buf_idx &
                            0xFF;
                        slave_devices[slave_idx].master_to_slave_buf[1] =
                            slave_devices[slave_idx].sending_arg_buf_idx >> 8 & 0xFF;
                        slave_devices[slave_idx].master_to_slave_buf[2] =
                            slave_devices[slave_idx].arg_buf[slave_devices[slave_idx].sending_arg_buf_idx - 1];
                    }
                }

                // if slave not ready before
                // but updated to ready in this cycle
                if (slave_devices[slave_idx].ready == 0 &&
                    slave_devices[slave_idx].arg_sent == 1 &&
                    slave_devices[slave_idx].slave_status == SLAVE_READY)
                {
                    // write initial value for each app
                    if (slave_devices[slave_idx].master_status != MASTER_READY)
                    {
                        slave_devices[slave_idx].master_to_slave_buf.clear();

                        // write init value
                        for (app_idx = 1; app_idx <= get_field_as<uint8_t>(
                                 slave_devices[slave_idx].sn, "task_count"); app_idx++)
                        {
                            // define outside loop to improve perf
                            // ReSharper disable once CppJoinDeclarationAndAssignment
                            task_type = get_field_as<uint8_t>(slave_devices[slave_idx].sn, app_idx,
                                                              "sdowrite_task_type");
                            // ReSharper disable once CppJoinDeclarationAndAssignment
                            pdo_offset = get_field_as<uint16_t>(slave_devices[slave_idx].sn, app_idx, "pdowrite_offset",
                                                                0);
                            offset = pdo_offset;

                            app_registry[task_type]->init_value(
                                slave_devices[slave_idx].master_to_slave_buf.data(),
                                &offset,
                                fmt::format("sn{}_app_{}_", slave_devices[slave_idx].sn, app_idx)
                            );
                        }

                        // after this slave will go into normal working state
                        RCLCPP_INFO(data_logger, "slave id %d sdo confirmed received", slave_idx);
                    }
                    slave_devices[slave_idx].master_status = MASTER_READY;
                }

                // if slave is ready/working
                if (slave_devices[slave_idx].ready == 1)
                {
                    // latency calc
                    // if the slaves works well
                    // master will continuous sending an incremental number in slave_devices[slave_idx].master_status
                    // and slave will send it back in slave_devices[slave_idx].slave_status
                    // the travel latency is the recv time - sent time
                    if (slave_devices[slave_idx].waiting_for_latency_checking == 1)
                    {
                        if (slave_devices[slave_idx].slave_status == slave_devices[slave_idx].master_status)
                        {
                            time_curr = rclcpp::Clock().now();
                            // unit: ms
                            std_msgs_float32_shared_msg.data = static_cast<float>(time_curr.seconds() - slave_devices[
                                    slave_idx].
                                last_packet_time.seconds()) * 1000.f;
                            slave_devices[slave_idx].last_packet_time = time_curr;
                            slave_devices[slave_idx].waiting_for_latency_checking = 0;
                            // publish latency data
                            publish_msg<std_msgs::msg::Float32>(
                                fmt::format("sn{}_latency_", slave_devices[slave_idx].sn),
                                std_msgs_float32_shared_msg);

                            // data processing
                            for (app_idx = 1; app_idx <= get_field_as<uint8_t>(
                                     slave_devices[slave_idx].sn, "task_count"); app_idx++)
                            {
                                // define outside loop to improve perf
                                // ReSharper disable once CppJoinDeclarationAndAssignment
                                task_type = get_field_as<uint8_t>(slave_devices[slave_idx].sn, app_idx,
                                                                  "sdowrite_task_type");
                                // ReSharper disable once CppJoinDeclarationAndAssignment
                                pdo_offset = get_field_as<uint16_t>(slave_devices[slave_idx].sn, app_idx,
                                                                    "pdoread_offset", 0);
                                offset = pdo_offset;

                                app_registry[task_type]->read(
                                    slave_devices[slave_idx].slave_to_master_buf.data(),
                                    &offset,
                                    fmt::format("sn{}_app_{}_", slave_devices[slave_idx].sn, app_idx)
                                );
                            }
                        }
                    }
                    else
                    {
                        // latency calculation number increments here
                        if (slave_devices[slave_idx].master_status >= 250)
                        {
                            slave_devices[slave_idx].master_status = MASTER_READY + 2;
                        }
                        else
                        {
                            slave_devices[slave_idx].master_status++;
                        }
                        slave_devices[slave_idx].waiting_for_latency_checking = 1;
                    }
                }
            }

            // transfer pdo data from buffer managed by ourselves info ecat stack
            for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
            {
                memcpy(ec_slave[slave_idx].outputs, &slave_devices[slave_idx].master_status, 1);
                memcpy(ec_slave[slave_idx].outputs + 1, slave_devices[slave_idx].master_to_slave_buf.data(),
                       slave_devices[slave_idx].master_to_slave_buf_len);
            }

            // send ecat frame
            ec_send_processdata();
        }
    }

    // destroy all publisher and subscriber
    sdo_data.reset_and_remove_publishers();
    RCLCPP_INFO(this->get_logger(), "DATA thread exiting...");
}

/**
 * slave sdo configuration write func
 * @param slave slave id
 * @return success or not
 */
// ReSharper disable once CppParameterMayBeConst
int config_ec_slave(ecx_contextt* /*context*/, uint16 slave)
{
    try
    {
        // read device sn
        ec_SDOread(slave, 0x1018, 4, FALSE, &sdo_size_ptr, &slave_devices[slave].sn, 0xffff);
        RCLCPP_INFO(cfg_logger, "Found slave id=%d, sn=%d, eepid=%d, type=%s", slave, slave_devices[slave].sn,
                    ec_slave[slave].eep_id, node->get_device_name(ec_slave[slave].eep_id).c_str());
        // write device sdo len
        sdo_size_write_ptr = get_field_as<uint16_t>(slave_devices[slave].sn, "sdo_len");
        ec_SDOwrite(slave, 0x8000, 0, FALSE, 2, &sdo_size_write_ptr, 0xffff);

        // init buffers
        slave_devices[slave].device_type = ec_slave[slave].eep_id;
        slave_devices[slave].master_status = sdo_size_write_ptr == 0 ? MASTER_READY : MASTER_SENDING_ARGUMENTS;

        slave_devices[slave].master_to_slave_buf.resize(
            node->get_device_master_to_slave_buf_len(ec_slave[slave].eep_id));
        slave_devices[slave].master_to_slave_buf_len = node->get_device_master_to_slave_buf_len(ec_slave[slave].eep_id);
        slave_devices[slave].master_to_slave_buf.clear();

        slave_devices[slave].slave_to_master_buf.resize(
            node->get_device_slave_to_master_buf_len(ec_slave[slave].eep_id));
        slave_devices[slave].slave_to_master_buf_len = node->get_device_slave_to_master_buf_len(ec_slave[slave].eep_id);
        slave_devices[slave].slave_to_master_buf.clear();

        RCLCPP_INFO(cfg_logger, "SDO configured for slave id=%d, sn=%d, eepid=%d, type=%s, sdolen=%d", slave,
                    slave_devices[slave].sn, ec_slave[slave].eep_id,
                    node->get_device_name(ec_slave[slave].eep_id).c_str(), sdo_size_write_ptr);

        slave_devices[slave].conf_done = 1;
        return 1;
    }
    catch (...)
    {
        return 0;
    }
}

bool EthercatNode::setup_ethercat(const char* ifname)
{
    if (!ec_init(ifname))
    {
        RCLCPP_ERROR(this->get_logger(), "No socket connection on %s. \n", ifname);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "ec_init on %s succeeded.", ifname);

    if (ec_config_init(FALSE) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No slaves found!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "%d slaves found", ec_slavecount);
    // write back to init state
    for (int i = 1; i <= ec_slavecount; i++)
    {
        ec_slave[i].state = EC_STATE_INIT;
        ec_writestate(i);
    }
    ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);

    RCLCPP_INFO(this->get_logger(), "all slaves backed to init, restarting mapping");
    const int reconf_slaves = ec_config_init(FALSE);
    RCLCPP_INFO(this->get_logger(), "detected %d slaves", reconf_slaves);
    // setup conf func
    for (int i = 1; i <= ec_slavecount; i++)
    {
        ec_slave[i].PO2SOconfigx = config_ec_slave;
        slave_devices[i].slave = &ec_slave[i];
    }

    // conf io map
    ec_config_map(&IOmap);
    ec_configdc();

    for (int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
        // create arg buffer
        slave_devices[slave_idx].arg_buf_len = get_field_as<uint16_t>(slave_devices[slave_idx].sn, "sdo_len");
        slave_devices[slave_idx].arg_buf.resize(slave_devices[slave_idx].arg_buf_len);
        slave_devices[slave_idx].arg_buf.clear();
        slave_devices[slave_idx].arg_buf_idx = 0;

        // create latency publisher for each module
        create_and_insert_publisher<std_msgs::msg::Float32>(
            fmt::format("sn{}_latency_", slave_devices[slave_idx].sn));

        // write basic args
        const uint8_t task_count = get_field_as<uint8_t>(slave_devices[slave_idx].sn, "task_count");
        slave_devices[slave_idx].arg_buf[slave_devices[slave_idx].arg_buf_idx++] = task_count;

        for (int app_idx = 1; app_idx <= task_count; app_idx++)
        {
            const uint8_t task_type = get_field_as<uint8_t>(slave_devices[slave_idx].sn, app_idx, "sdowrite_task_type");
            // write basic args
            memcpy(slave_devices[slave_idx].arg_buf.data() + slave_devices[slave_idx].arg_buf_idx,
                   sdo_data.build_buf(
                       fmt::format("sn{}_app_{}_sdowrite_", slave_devices[slave_idx].sn, app_idx),
                       {
                           "task_type"
                       }),
                   1);
            slave_devices[slave_idx].arg_buf_idx++;

            // call init func of each app/task
            app_registry[task_type]->init_sdo(
                slave_devices[slave_idx].arg_buf.data(),
                &slave_devices[slave_idx].arg_buf_idx,
                slave_devices[slave_idx].sn,
                slave_idx,
                fmt::format("sn{}_app_{}_", slave_devices[slave_idx].sn, app_idx)
            );
        }

        // mark ros conf step as done
        slave_devices[slave_idx].conf_ros_done = 1;
    }

    // change slave state to op
    RCLCPP_INFO(this->get_logger(), "Slaves mapped, state to SAFE_OP.");
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    RCLCPP_INFO(this->get_logger(), "All slaves reached SAFE_OP, state to OP");
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    // mock data process
    int chk = 50;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);
    }
    while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

    // pre-final-op state check
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        RCLCPP_INFO(this->get_logger(), "Operational state reached for all slaves.");
        pdo_transfer_active = true;
        worker_thread_ = std::thread(&EthercatNode::datacycle_callback, this);
        return true;
    }

    return true;
}

/**
 * deinit EtherCat sys
 */
EthercatNode::~EthercatNode()
{
    RCLCPP_INFO(this->get_logger(), "Stop data cycle");
    pdo_transfer_active = false;
    running_ = false;
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "Request init state for all slaves");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}

void EthercatNode::register_components()
{
    register_module(1, "FlightModule", 16, 40);
    register_module(2, "MotorModule", 56, 80);
    register_module(3, "H750UniversalModule", 80, 80);

    register_app<DJIRC>();
    register_app<HIPNUC_IMU>();
    register_app<DJI_MOTOR>();
    register_app<DSHOT>();
    register_app<VANILLA_PWM>();
    register_app<EXTERNAL_PWM>();
    register_app<MS5876_30BA>();
    register_app<LK_MOTOR>();
    register_app<ADC>();
    register_app<CAN_PMU>();
    register_app<SBUSRC>();
    register_app<DM_MOTOR>();
}

int main(const int argc, const char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<EthercatNode>();
    if (node->setup_ethercat(node->interface.c_str()))
    {
        RCLCPP_INFO(cfg_logger, "Initialization succeeded");
        rclcpp::spin(node);
        node.reset();
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_ERROR(cfg_logger, "Initialization failed");
        node.reset();
        rclcpp::shutdown();
    }
    return 0;
}
