//
// Created by hang on 10/10/26.
//
#include "soem_wrapper/device/aim_universal_module.hpp"
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"
#include "soem_wrapper/utils/config_utils.hpp"

#include "ethercat.h"

namespace aim::ecat::device {
    using namespace logging;
    using namespace utils::config;

    AimUniversalModule::AimUniversalModule(const DeviceInfo &info, const AimModuleSpec &spec)
        : SlaveDevice(info), spec_(spec) {
    }

    AimUniversalModule::~AimUniversalModule() = default;

    int AimUniversalModule::on_configure(ConfigContext &ctx) {
        try {
            uint32_t old_sn;
            {
                std::lock_guard lock(mtx_);
                old_sn = sn_;
            }

            // read device info
            uint32_t sn = old_sn;
            int sdo_sn_size = sizeof(sn);
            ec_SDOread(get_index(), 0x1018, 4, FALSE, &sdo_sn_size, &sn, 0xffff);

            char sw_rev_str[4]{};
            int sdo_rev_size = 3;
            ec_SDOread(get_index(), 0x100A, 0, FALSE, &sdo_rev_size, &sw_rev_str, 0xffff);

            const int sw_rev = atoi(sw_rev_str); // NOLINT

            if (old_sn != 0) {
                RCLCPP_INFO_THROTTLE(*get_cfg_logger(),
                                     *get_node()->get_clock(), 1500,
                                     "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                                     get_index(),
                                     sn,
                                     get_product_code(),
                                     get_name().c_str(),
                                     sw_rev);
            } else {
                RCLCPP_INFO(*get_cfg_logger(),
                            "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                            get_index(),
                            sn,
                            get_product_code(),
                            get_name().c_str(),
                            sw_rev);
            }

            if (old_sn != 0 && old_sn != sn) {
                RCLCPP_ERROR_THROTTLE(*get_cfg_logger(),
                                      *get_node()->get_clock(),
                                      1500,
                                      "Slave idx=%d reconnected with different board, config rejected",
                                      get_index());
                std::lock_guard lock(mtx_);
                is_recover_rejected_ = true;
                return 0;
            }

            {
                std::lock_guard lock(mtx_);
                sn_ = sn;
                sw_rev_ = sw_rev;
                is_recover_rejected_ = false;
                set_config_prefix(fmt::format("sn{}_", sn_));

                if (sw_rev_ < spec_.min_sw_rev) {
                    RCLCPP_ERROR(
                        *get_cfg_logger(),
                        "Slave id=%d, sn=%d, swrev=%d, don't meet the requirement of minimum sw rev=%d, please flash the newest firmware.",
                        get_index(), sn_,
                        sw_rev_,
                        spec_.min_sw_rev);
                    return 0;
                }
                is_sw_rev_check_passed_ = true;
            }

            // write device sdo len
            auto sdo_size_write = get_field_as<uint16_t>(
                ctx.config,
                sn,
                "sdo_len");
            ec_SDOwrite(get_index(), 0x8000, 0, FALSE, 2, &sdo_size_write, 0xffff);

            std::lock_guard lock(mtx_);

            // init buffers
            master_status_ = sdo_size_write == 0 ? MASTER_READY : MASTER_SENDING_ARGUMENTS;

            // only explicitly clear buf in first configuration try
            if (old_sn == 0) {
                master_to_slave_buf_.clear();
                master_to_slave_buf_.resize(spec_.master_to_slave_buf_len);
                master_to_slave_buf_len_ = spec_.master_to_slave_buf_len;

                slave_to_master_buf_.clear();
                slave_to_master_buf_.resize(spec_.slave_to_master_buf_len);
                slave_to_master_buf_len_ = spec_.slave_to_master_buf_len;
            }

            RCLCPP_INFO(*get_cfg_logger(),
                        "SDO configured for slave id=%d, sn=%d, eepid=%d, type=%s, sdolen=%d",
                        get_index(),
                        sn_,
                        get_product_code(),
                        get_name().c_str(), sdo_size_write);

            is_ecat_conf_done_ = true;
            return 1;
        } catch (...) {
            return 0;
        }
    }

    bool AimUniversalModule::on_setup(SetupContext &ctx) {
        std::lock_guard lock(mtx_);

        if (!is_sw_rev_check_passed_) {
            return false;
        }

        // create arg buffer
        arg_buf_len_ = get_field_as<uint16_t>(
            ctx.config,
            sn_,
            "sdo_len");
        arg_buf_.clear();
        arg_buf_.resize(arg_buf_len_);

        int arg_buf_idx = 0;
        // create latency publisher for each module
        latency_publisher_ = ctx.node.create_publisher<std_msgs::msg::Float32>(
            get_field_as<std::string>(
                ctx.config,
                fmt::format("sn{}_latency_pub_topic", sn_)),
            rclcpp::SensorDataQoS()
        );

        // write basic args
        const uint8_t task_count = get_field_as<uint8_t>( // NOLINT(*-use-auto)
            ctx.config,
            sn_,
            "task_count");

        if (task_count == 0) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave %d don't have any task, please add at least one or remove this slave",
                         get_index());
            return false;
        }
        arg_buf_[arg_buf_idx++] = task_count;

        task_list_.clear();
        task_list_.reserve(task_count);
        for (int app_idx = 1; app_idx <= task_count; app_idx++) {
            const uint8_t task_type = get_field_as<uint8_t>( // NOLINT(*-use-auto)
                ctx.config,
                sn_,
                app_idx,
                "sdowrite_task_type");
            // write basic args
            auto [sdo_buf, sdo_len] = ctx.config.build_buf(
                fmt::format("sn{}_app_{}_sdowrite_",
                            sn_,
                            app_idx),
                {
                    "task_type"
                }
            );
            memcpy(arg_buf_.data()
                   + arg_buf_idx,
                   sdo_buf,
                   sdo_len);
            arg_buf_idx++;

            std::unique_ptr<task::TaskWrapper> task_wrapper{};

            switch (task_type) {
                case task::DJIRC_APP_ID: {
                    task_wrapper = std::make_unique<task::dbus_rc::DBUS_RC>();
                    break;
                }
                case task::LK_APP_ID: {
                    task_wrapper = std::make_unique<task::lk_motor::LK_MOTOR>();
                    break;
                }
                case task::HIPNUC_IMU_CAN_APP_ID: {
                    task_wrapper = std::make_unique<task::hipnuc_imu::HIPNUC_IMU_CAN>();
                    break;
                }
                case task::DSHOT_APP_ID: {
                    task_wrapper = std::make_unique<task::pwm::DSHOT>();
                    break;
                }
                case task::DJICAN_APP_ID: {
                    task_wrapper = std::make_unique<task::dji_motor::DJI_MOTOR>();
                    break;
                }
                case task::ONBOARD_PWM_APP_ID: {
                    task_wrapper = std::make_unique<task::pwm::ONBOARD_PWM>();
                    break;
                }
                case task::CAN_PMU_APP_ID: {
                    task_wrapper = std::make_unique<task::pmu_uavcan::PMU_CAN>();
                    break;
                }
                case task::SBUS_RC_APP_ID: {
                    task_wrapper = std::make_unique<task::sbus_rc::SBUS_RC>();
                    break;
                }
                case task::DM_MOTOR_APP_ID: {
                    task_wrapper = std::make_unique<task::dm_motor::DM_MOTOR>();
                    break;
                }
                case task::DM_MOTOR_CANFD_APP_ID: {
                    task_wrapper = std::make_unique<task::dm_motor::DM_MOTOR_CANFD>();
                    break;
                }
                case task::SUPER_CAP_APP_ID: {
                    task_wrapper = std::make_unique<task::super_cap::SUPER_CAP>();
                    break;
                }
                case task::VT13_RC_APP_ID: {
                    task_wrapper = std::make_unique<task::vt13_rc::VT13_RC>();
                    break;
                }
                case task::EXTERNAL_PWM_APP_ID: {
                    task_wrapper = std::make_unique<task::pwm::EXTERNAL_PWM>();
                    break;
                }
                case task::MS5837_30BA_APP_ID: {
                    task_wrapper = std::make_unique<task::ms5837::MS5837_30BA>();
                    break;
                }
                case task::DD_MOTOR_APP_ID: {
                    task_wrapper = std::make_unique<task::dd_motor::DD_MOTOR>();
                    break;
                }
                default: {
                    RCLCPP_ERROR(*get_cfg_logger(),
                                 "Unknown task type = %d, slave id=%d, app idx=%d",
                                 task_type,
                                 get_index(),
                                 app_idx);
                    return false;
                }
            }

            task_wrapper->init_sdo(arg_buf_.data(),
                                   &arg_buf_idx,
                                   get_index(),
                                   fmt::format("sn{}_app_{}_", sn_, app_idx)
            );
            task_list_.push_back(std::move(task_wrapper));
        }

        // mark ros conf step as done
        is_conf_ros_done_ = true;

        return true;
    }

    void AimUniversalModule::on_receive() {
        memcpy(&slave_status_, ec_slave[get_index()].inputs, 1);
        memcpy(slave_to_master_buf_.data(), ec_slave[get_index()].inputs + 1,
               slave_to_master_buf_len_);
    }

    void AimUniversalModule::on_cycle(const CycleContext &ctx) {
        // slave report that all args are well-received
        if (slave_status_ == SLAVE_CONFIRM_READY && !is_ready_) {
            RCLCPP_INFO(*get_data_logger(), "Slave id=%d confirmed ready", get_index());
            is_ready_ = true;
        }

        // sending args
        // master will send arg bytes one by one
        // slave will send what it receives back to the master
        // to ensure the data is correct
        if (master_status_ == MASTER_SENDING_ARGUMENTS && !is_arg_sent_) {
            send_arg();
        }

        // if slave not ready before
        // but updated to ready in this cycle
        if (!is_ready_ && is_arg_sent_ && slave_status_ == SLAVE_READY) {
            // write initial value for each app
            // only write in first initialization
            if (master_status_ != MASTER_READY) {
                if (reconnected_times_ == 0) {
                    write_init_values();
                } else {
                    recover_master_to_slave_buf();
                    RCLCPP_INFO(*get_health_checker_logger(),
                                "Slave id=%d master to slave buf recovered", get_index());
                }

                // after this slave will go into normal working state
                RCLCPP_INFO(*get_data_logger(),
                            "Slave id=%d sdo confirmed received", get_index());
            }

            master_status_ = MASTER_READY;
        }

        // if slave is ready/working
        if (is_ready_) {
            process_pdo(ctx.now);
        }
    }

    void AimUniversalModule::on_transmit() {
        memcpy(ec_slave[get_index()].outputs, &master_status_, 1);
        memcpy(ec_slave[get_index()].outputs + 1, master_to_slave_buf_.data(),
               master_to_slave_buf_len_);
    }

    void AimUniversalModule::send_arg() {
        // all arg bytes are confirmed
        if ((static_cast<uint16_t>(slave_to_master_buf_[1]) << 8 | slave_to_master_buf_[0]) == sending_arg_buf_idx_
            && slave_to_master_buf_[2] == arg_buf_[sending_arg_buf_idx_ - 1]) {
            if (sending_arg_buf_idx_ == arg_buf_len_) {
                RCLCPP_INFO(*get_data_logger(), "Slave id=%d sdo all sent", get_index());
                is_arg_sent_ = true;
            } else {
                sending_arg_buf_idx_++;
            }
        }

        // sending arg bytes
        if (!is_arg_sent_) {
            RCLCPP_DEBUG(*get_data_logger(),
                         "Slave id=%d sending sdo idx %d / %d, content= %d, "
                         "slavestatus=%d, masterstatus=%d",
                         get_index(),
                         sending_arg_buf_idx_,
                         arg_buf_len_,
                         arg_buf_[sending_arg_buf_idx_ - 1],
                         slave_status_,
                         master_status_);

            master_to_slave_buf_[0] = sending_arg_buf_idx_ & 0xFF;
            master_to_slave_buf_[1] = sending_arg_buf_idx_ >> 8 & 0xFF;
            master_to_slave_buf_[2] = arg_buf_[sending_arg_buf_idx_ - 1];
        }
    }

    void AimUniversalModule::process_pdo(const rclcpp::Time &current_time) {
        // latency calc
        // if the slaves works well
        // master will continuous sending an incremental number in master_status
        // and slave will send it back in slave_status
        // the travel latency is the recv time - sent time
        if (slave_status_ == master_status_) {
            // slave is back in normal working state, the backup is outdated from now on
            is_backup_valid_ = false;

            // unit: ms
            std_msgs_float32_shared_msg.data =
                    static_cast<float>((current_time - last_latency_check_packet_send_time_).seconds() * 1000.f);
            current_data_stamp_ = last_latency_check_packet_send_time_;
            last_latency_check_packet_send_time_ = current_time;
            latency_publisher_->publish(std_msgs_float32_shared_msg);

            // latency calculation number increments here
            if (master_status_ >= 250) {
                master_status_ = MASTER_READY + 2;
            } else {
                master_status_++;
            }

            for (const auto &task: task_list_) {
                if (task->has_publishers()) {
                    task->read();
                }
            }
        }
    }

    void AimUniversalModule::write_init_values() {
        memset(master_to_slave_buf_.data(), 0, master_to_slave_buf_len_);
        for (const auto &task: task_list_) {
            if (task->has_subscribers()) {
                task->init_value();
            }
        }
    }

    void AimUniversalModule::write_safe_outputs() {
        // before all args are sent, the buffer is used for arg transmission
        if (is_arg_sent_) {
            write_init_values();
        }
    }

    void AimUniversalModule::recover_master_to_slave_buf() {
        if (master_to_slave_buf_backup_.size() != master_to_slave_buf_len_) {
            return;
        }

        memcpy(
            master_to_slave_buf_.data(),
            master_to_slave_buf_backup_.data(),
            master_to_slave_buf_len_
        );
        is_backup_valid_ = false;
    }

    void AimUniversalModule::backup_master_to_slave_buf() {
        master_to_slave_buf_backup_.clear();
        master_to_slave_buf_backup_.resize(
            master_to_slave_buf_len_);
        memcpy(
            master_to_slave_buf_backup_.data(),
            master_to_slave_buf_.data(),
            master_to_slave_buf_len_
        );
        is_backup_valid_ = true;
    }

    bool AimUniversalModule::is_configured() const {
        return is_conf_ros_done_ && is_ecat_conf_done_;
    }

    bool AimUniversalModule::is_ready() const {
        return is_ready_ && slave_status_ >= SLAVE_CONFIRM_READY;
    }

    void AimUniversalModule::pre_recover() {
        // slave went back to INIT without being detected as lost,
        // take a backup now before send_arg overwrites the buffer
        if (!is_backup_valid_) {
            backup_master_to_slave_buf();
        }

        is_arg_sent_ = false;
        sending_arg_buf_idx_ = 1;
        reconnected_times_++;
        is_conf_ros_done_ = true;
    }

    void AimUniversalModule::reset_state() {
        is_ready_ = false;
        is_conf_ros_done_ = false;
        slave_status_ = SLAVE_INITIALIZING;
    }

    void AimUniversalModule::recover_state() {
        is_ready_ = true;
        is_conf_ros_done_ = true;
    }

    void AimUniversalModule::on_connection_lost() {
        for (const auto &task: task_list_) {
            // process mst to slv connection lost actions
            // 0x02 = Reset to default
            if (task->get_connection_lost_write_action() == 0x02) {
                task->init_value();
                RCLCPP_WARN(*get_health_checker_logger(),
                            "Slave idx=%d, task=%s, control command has been reset",
                            get_index(),
                            task->get_type_name().c_str());
            }
            // process slv to mst connection lost actions
            if (task->get_connection_lost_read_action() == 0x02) {
                task->publish_empty_message();
                RCLCPP_WARN(*get_health_checker_logger(),
                            "Slave idx=%d, task=%s, report message has been reset",
                            get_index(),
                            task->get_type_name().c_str());
            }
        }

        // backup after the reset actions, restored when the args are resent
        backup_master_to_slave_buf();
    }

    bool AimUniversalModule::is_recover_rejected() const {
        return is_recover_rejected_;
    }

    void AimUniversalModule::cleanup() {
        if (latency_publisher_) {
            latency_publisher_.reset();
        }
        for (const auto &task: task_list_) {
            task->cleanup();
        }
    }
}
