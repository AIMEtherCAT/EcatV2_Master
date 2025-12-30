//
// Created by hang on 25-4-21.
//
#include "soem_wrapper/wrapper.hpp"
#include "soem_wrapper/task_defs.hpp"
#include "soem_wrapper/ecat_node.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"
#include "soem_wrapper/utils/config_utils.hpp"

#include "mutex"
#include "ethercat.h"

namespace aim::ecat {
    using namespace logging;
    using namespace utils::config;

    static std::vector<std::shared_ptr<SlaveDevice> > slave_devices;

    void init_slave_devices_vector(const int slave_count) {
        slave_devices.clear();
        slave_devices.reserve(slave_count);
        for (int i = 1; i <= slave_count; i++) {
            slave_devices.push_back(std::make_shared<SlaveDevice>(i));
        }
    }

    std::shared_ptr<SlaveDevice> get_slave_device(const int index) {
        assert(index >= 1);
        return slave_devices.at(index - 1);
    }

    std::vector<std::shared_ptr<SlaveDevice> > &get_slave_devices() {
        return slave_devices;
    }

    SlaveDevice::SlaveDevice(const uint16_t index) {
        index_ = index;
    }

    SlaveDevice::~SlaveDevice() = default;

    int SlaveDevice::init() {
        try {
            std::lock_guard lock(mtx_);
            const uint32_t oldSn = sn_;

            // read device info
            sdo_sn_size_ptr_ = 4;
            ec_SDOread(index_, 0x1018, 4, FALSE, &sdo_sn_size_ptr_, &sn_, 0xffff);

            sdo_rev_size_ptr_ = 3;
            ec_SDOread(index_, 0x100A, 0, FALSE, &sdo_rev_size_ptr_, &sw_rev_str_, 0xffff);

            sw_rev_ = atoi(sw_rev_str_); // NOLINT

            if (oldSn != 0) {
                RCLCPP_INFO_THROTTLE(*get_cfg_logger(),
                                     *get_node()->get_clock(), 1500,
                                     "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                                     index_,
                                     sn_,
                                     ec_slave[index_].eep_id,
                                     get_node()->get_device_name(ec_slave[index_].eep_id).c_str(),
                                     sw_rev_);
            } else {
                RCLCPP_INFO(*get_cfg_logger(),
                            "Found slave id=%d, sn=%d, eepid=%d, type=%s, swrev=%d",
                            index_,
                            sn_,
                            ec_slave[index_].eep_id,
                            get_node()->get_device_name(ec_slave[index_].eep_id).c_str(),
                            sw_rev_);
            }

            if (oldSn != 0 && oldSn != sn_) {
                RCLCPP_ERROR_THROTTLE(*get_cfg_logger(),
                                      *get_node()->get_clock(),
                                      1500,
                                      "Slave idx=%d reconnected with different board, config rejected",
                                      index_);
                sn_ = oldSn;
                is_recover_rejected_ = true;
                return 0;
            }
            is_recover_rejected_ = false;

            if (sw_rev_ < get_node()->get_device_min_sw_rev_requirement(ec_slave[index_].eep_id)) {
                RCLCPP_ERROR(
                    *get_cfg_logger(),
                    "Slave id=%d, sn=%d, swrev=%d, don't meet the requirement of minimum sw rev=%d, please flash the newest firmware.",
                    index_, sn_,
                    sw_rev_,
                    get_node()->get_device_min_sw_rev_requirement(ec_slave[index_].eep_id));
                return 0;
            }
            is_sw_rev_check_passed_ = true;

            // write device sdo len
            sdo_size_write_ptr_ = get_field_as<uint16_t>(
                *get_configuration_data(),
                sn_,
                "sdo_len");
            ec_SDOwrite(index_, 0x8000, 0, FALSE, 2, &sdo_size_write_ptr_, 0xffff);

            // init buffers
            device_type = ec_slave[index_].eep_id;
            master_status_ = sdo_size_write_ptr_ == 0 ? MASTER_READY : MASTER_SENDING_ARGUMENTS;

            // only explicitly clear buf in first configuration try
            if (oldSn == 0) {
                master_to_slave_buf_.clear();
                master_to_slave_buf_.resize(
                    get_node()->get_device_master_to_slave_buf_len(ec_slave[index_].eep_id));
                master_to_slave_buf_len_ = get_node()->get_device_master_to_slave_buf_len(
                    ec_slave[index_].eep_id);
                master_to_slave_buf_.clear();

                slave_to_master_buf_.clear();
                slave_to_master_buf_.resize(
                    get_node()->get_device_slave_to_master_buf_len(ec_slave[index_].eep_id));
                slave_to_master_buf_len_ = get_node()->get_device_slave_to_master_buf_len(
                    ec_slave[index_].eep_id);
                slave_to_master_buf_.clear();
            }

            RCLCPP_INFO(*get_cfg_logger(),
                        "SDO configured for slave id=%d, sn=%d, eepid=%d, type=%s, sdolen=%d",
                        index_,
                        sn_,
                        ec_slave[index_].eep_id,
                        get_node()->get_device_name(ec_slave[index_].eep_id).c_str(), sdo_size_write_ptr_);

            is_ecat_conf_done_ = true;
            return 1;
        } catch (...) {
            return 0;
        }
    }

    bool SlaveDevice::init_task_list() {
        std::lock_guard lock(mtx_);

        if (!is_sw_rev_check_passed_) {
            return false;
        }

        // create arg buffer
        arg_buf_len_ = get_field_as<uint16_t>(
            *get_configuration_data(),
            sn_,
            "sdo_len");
        arg_buf_.clear();
        arg_buf_.resize(arg_buf_len_);

        int arg_buf_idx = 0;
        // create latency publisher for each module
        latency_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(
            get_field_as<std::string>(
                *get_configuration_data(),
                fmt::format("sn{}_latency_pub_topic", sn_)),
            rclcpp::SensorDataQoS()
        );

        // write basic args
        const uint8_t task_count = get_field_as<uint8_t>( // NOLINT(*-use-auto)
            *get_configuration_data(),
            sn_,
            "task_count");

        if (task_count == 0) {
            RCLCPP_ERROR(*logging::get_cfg_logger(),
                         "Slave %d don't have any task, please add at least one or remove this slave",
                         index_);
            return false;
        }
        arg_buf_[arg_buf_idx++] = task_count;

        task_list_.clear();
        task_list_.reserve(task_count);
        for (int app_idx = 1; app_idx <= task_count; app_idx++) {
            const uint8_t task_type = get_field_as<uint8_t>( // NOLINT(*-use-auto)
                *get_configuration_data(),
                sn_,
                app_idx,
                "sdowrite_task_type");
            // write basic args
            auto [sdo_buf, sdo_len] = get_configuration_data()->build_buf(
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
                case task::SBUS_RC_APP_ID: {
                    task_wrapper = std::make_unique<task::sbus_rc::SBUS_RC>();
                    break;
                }
                case task::DM_MOTOR_APP_ID: {
                    task_wrapper = std::make_unique<task::dm_motor::DM_MOTOR>();
                    break;
                }
                default: {
                    RCLCPP_ERROR(*get_cfg_logger(), "Unknown task type = %d", task_type);
                }
            }

            task_wrapper->init_sdo(arg_buf_.data(),
                                   &arg_buf_idx,
                                   index_,
                                   fmt::format("sn{}_app_{}_", sn_, app_idx)
            );
            task_list_.push_back(std::move(task_wrapper));
        }

        // mark ros conf step as done
        is_conf_ros_done_ = true;

        return true;
    }

    void SlaveDevice::transfer_to_slave() const {
        memcpy(ec_slave[index_].outputs, &master_status_, 1);
        memcpy(ec_slave[index_].outputs + 1, master_to_slave_buf_.data(),
               master_to_slave_buf_len_);
    }

    void SlaveDevice::receive_from_slave() {
        memcpy(&slave_status_, ec_slave[index_].inputs, 1);
        memcpy(slave_to_master_buf_.data(), ec_slave[index_].inputs + 1,
               slave_to_master_buf_len_);
    }

    void SlaveDevice::send_arg() {
        // all arg bytes are confirmed
        if ((static_cast<uint16_t>(slave_to_master_buf_[1]) << 8 | slave_to_master_buf_[0]) == sending_arg_buf_idx_
            && slave_to_master_buf_[2] == arg_buf_[sending_arg_buf_idx_ - 1]) {
            if (sending_arg_buf_idx_ == arg_buf_len_) {
                RCLCPP_INFO(*logging::get_data_logger(), "Slave id=%d sdo all sent", index_);
                is_arg_sent_ = true;
            } else {
                sending_arg_buf_idx_++;
            }
        }

        // sending arg bytes
        if (!is_arg_sent_) {
            RCLCPP_DEBUG(*logging::get_data_logger(),
                         "Slave id=%d sending sdo idx %d / %d, content= %d, "
                         "slavestatus=%d, masterstatus=%d",
                         index_,
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

    void SlaveDevice::process_pdo(const rclcpp::Time &current_time) {
        // latency calc
        // if the slaves works well
        // master will continuous sending an incremental number in master_status
        // and slave will send it back in slave_status
        // the travel latency is the recv time - sent time
        if (slave_status_ == master_status_) {
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

    void SlaveDevice::write_init_values() {
        memset(master_to_slave_buf_.data(), 0, master_to_slave_buf_len_);
        for (const auto &task: task_list_) {
            if (task->has_publishers()) {
                task->init_value();
            }
        }
    }

    void SlaveDevice::recover_master_to_slave_buf() {
        memcpy(
            master_to_slave_buf_.data(),
            master_to_slave_buf_backup_.data(),
            master_to_slave_buf_len_
        );
    }

    void SlaveDevice::pre_recover() {
        is_arg_sent_ = false;
        sending_arg_buf_idx_ = 1;
        reconnected_times_++;
        is_conf_ros_done_ = true;
    }

    void SlaveDevice::reset_state() {
        is_ready_ = false;
        is_conf_ros_done_ = false;
        slave_status_ = SLAVE_INITIALIZING;
    }

    void SlaveDevice::recover_state() {
        is_ready_ = true;
        is_conf_ros_done_ = true;
    }

    void SlaveDevice::backup_master_to_slave_buf() {
        master_to_slave_buf_backup_.clear();
        master_to_slave_buf_backup_.resize(
            master_to_slave_buf_len_);
        memcpy(
            master_to_slave_buf_backup_.data(),
            master_to_slave_buf_.data(),
            master_to_slave_buf_len_
        );
    }

    void SlaveDevice::on_connection_lost() const {
        for (const auto &task: task_list_) {
            // process mst to slv connection lost actions
            // 0x02 = Reset to default
            if (task->get_connection_lost_write_action() == 0x02) {
                task->init_value();
                RCLCPP_WARN(*logging::get_health_checker_logger(),
                            "Slave idx=%d, task=%s, control command has been reset",
                            index_,
                            task->get_type_name().c_str());
            }
            // process slv to mst connection lost actions
            if (task->get_connection_lost_read_action() == 0x02) {
                task->publish_empty_message();
                RCLCPP_WARN(*logging::get_health_checker_logger(),
                            "Slave idx=%d, task=%s, report message has been reset",
                            index_,
                            task->get_type_name().c_str());
            }
        }
    }

    void SlaveDevice::cleanup() {
        if (latency_publisher_) {
            latency_publisher_.reset();
        }
        for (const auto &task: task_list_) {
            task->cleanup();
        }
    }

    /**
     * slave sdo configuration write func
     * @param slave slave id
     * @return success or not
     */
    // ReSharper disable once CppParameterMayBeConst
    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave) {
        return get_slave_device(slave)->init();
    }

    void run() {
        if (get_node()->setup_ecat()) {
            RCLCPP_INFO(*get_sys_logger(), "Initialization succeeded");
            rclcpp::on_shutdown([&] {
                get_node()->on_shutdown();
            });
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
