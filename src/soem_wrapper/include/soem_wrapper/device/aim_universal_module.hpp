//
// Created by hang on 10/10/26.
//

#ifndef SOEM_WRAPPER_DEVICE_AIM_UNIVERSAL_MODULE_HPP
#define SOEM_WRAPPER_DEVICE_AIM_UNIVERSAL_MODULE_HPP

#include "soem_wrapper/device/slave_device.hpp"
#include "std_msgs/msg/float32.hpp"

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
}

namespace aim::ecat::device {
    constexpr uint32_t AIM_VENDOR_ID = 0x00000000;

    struct AimModuleSpec {
        uint16_t master_to_slave_buf_len;
        uint16_t slave_to_master_buf_len;
        int min_sw_rev;
    };

    class AimUniversalModule final : public SlaveDevice {
    public:
        AimUniversalModule(const DeviceInfo &info, const AimModuleSpec &spec);

        ~AimUniversalModule() override;

        int on_configure(ConfigContext &ctx) override;

        bool on_setup(SetupContext &ctx) override;

        void on_op_reached() override {
            // ready state is decided by the arg handshake
        }

        void on_receive() override;

        void on_cycle(const CycleContext &ctx) override;

        void on_transmit() override;

        void write_safe_outputs() override;

        [[nodiscard]] bool is_configured() const override;

        [[nodiscard]] bool is_ready() const override;

        void pre_recover() override;

        void reset_state() override;

        void recover_state() override;

        void on_connection_lost() override;

        [[nodiscard]] bool is_recover_rejected() const override;

        void cleanup() override;

        std::vector<uint8_t> &get_master_to_slave_buf() {
            return master_to_slave_buf_;
        }

        std::vector<uint8_t> &get_slave_to_master_buf() {
            return slave_to_master_buf_;
        }

        rclcpp::Time &get_current_data_stamp() {
            return current_data_stamp_;
        }

    private:
        void send_arg();

        void process_pdo(const rclcpp::Time &current_time);

        void write_init_values();

        void recover_master_to_slave_buf();

        void backup_master_to_slave_buf();

        inline static std_msgs::msg::Float32 std_msgs_float32_shared_msg{};

        AimModuleSpec spec_;

        // slave info
        int sw_rev_ = 0;
        uint32_t sn_ = 0;

        // slave status
        bool is_sw_rev_check_passed_{false};
        bool is_recover_rejected_{false};
        bool is_ready_{false};

        rclcpp::Time last_latency_check_packet_send_time_{};
        rclcpp::Time current_data_stamp_{};

        // master to slave
        uint8_t master_status_ = 0;
        std::vector<uint8_t> master_to_slave_buf_{};
        // used for connection lost recover
        std::vector<uint8_t> master_to_slave_buf_backup_{};
        // backup holds the commands to restore after the args are resent
        bool is_backup_valid_{false};
        uint16_t master_to_slave_buf_len_ = 0;

        // slave to master
        uint8_t slave_status_ = 0;
        std::vector<uint8_t> slave_to_master_buf_{};
        uint16_t slave_to_master_buf_len_ = 0;

        // sdo configuration
        std::vector<uint8_t> arg_buf_{};
        uint16_t arg_buf_len_ = 0;
        uint16_t sending_arg_buf_idx_ = 1;
        bool is_arg_sent_{false};

        // ecat conf done flag
        bool is_ecat_conf_done_{false};
        // ros2 pub/sub conf done flag
        bool is_conf_ros_done_{false};

        uint8_t reconnected_times_ = 0;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_publisher_{};

        std::vector<std::unique_ptr<task::TaskWrapper> > task_list_;
    };
}

#endif //SOEM_WRAPPER_DEVICE_AIM_UNIVERSAL_MODULE_HPP
