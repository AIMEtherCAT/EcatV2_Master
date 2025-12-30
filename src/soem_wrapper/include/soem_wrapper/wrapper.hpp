//
// Created by hang on 12/26/25.
//

#ifndef BUILD_SOEM_WRAPPER_H
#define BUILD_SOEM_WRAPPER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

typedef uint16_t uint16;
typedef struct ecx_context ecx_contextt;

namespace aim::ecat::task {
    class TaskWrapper;
}

namespace aim::ecat {
    class SlaveDevice final {
    public:
        explicit SlaveDevice(uint16_t index);

        ~SlaveDevice();

        int init();

        bool init_task_list();

        void transfer_to_slave() const;

        void receive_from_slave();

        void send_arg();

        void process_pdo(const rclcpp::Time &current_time);

        void write_init_values();

        void recover_master_to_slave_buf();

        void pre_recover();

        void reset_state();

        void recover_state();

        void on_connection_lost() const;

        void backup_master_to_slave_buf();

        std::vector<uint8_t> &get_master_to_slave_buf() {
            return master_to_slave_buf_;
        }

        std::vector<uint8_t> &get_slave_to_master_buf() {
            return slave_to_master_buf_;
        }

        std::vector<uint8_t> &get_arg_buf() {
            return arg_buf_;
        }

        rclcpp::Time &get_current_data_stamp() {
            return current_data_stamp_;
        }

        uint8_t *get_slave_status_ptr() { return &slave_status_; }

        uint8_t *get_master_status_ptr() { return &master_status_; }

        uint16_t get_master_to_slave_buf_len() const {
            return master_to_slave_buf_len_;
        }

        uint16_t get_slave_to_master_buf_len() const {
            return slave_to_master_buf_len_;
        }

        uint16_t get_arg_buf_len() const {
            return arg_buf_len_;
        }

        bool is_ready() const {
            return is_ready_;
        }

        bool is_ecat_conf_done() const {
            return is_ecat_conf_done_;
        }

        bool is_conf_ros_done() const {
            return is_conf_ros_done_;
        }

        void set_ready(const bool val) {
            is_ready_ = val;
        }

        bool is_arg_sent() const {
            return is_arg_sent_;
        }

        uint8_t get_reconnected_times() const {
            return reconnected_times_;
        }

        uint16_t get_index() const {
            return index_;
        }

        bool is_recover_rejected() const {
            return is_recover_rejected_;
        }

        void cleanup();

        mutable std::mutex mtx_;

    private:
        inline static std_msgs::msg::Float32 std_msgs_float32_shared_msg{};

        // shared static vars
        inline static int sdo_sn_size_ptr_ = 4;
        inline static int sdo_rev_size_ptr_ = 3;
        inline static char sw_rev_str_[4]{};
        inline static uint16_t sdo_size_write_ptr_ = 0;

        // slave info
        uint16_t index_ = 0;
        int sw_rev_ = 0;
        uint32_t sn_ = 0;
        uint8_t device_type = 0;

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

    void init_slave_devices_vector(int slave_count);

    std::shared_ptr<SlaveDevice> get_slave_device(int index);

    std::vector<std::shared_ptr<SlaveDevice> > &get_slave_devices();

    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave);
}

#endif //BUILD_SOEM_WRAPPER_H