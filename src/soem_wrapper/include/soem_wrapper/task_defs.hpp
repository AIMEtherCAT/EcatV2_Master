//
// Created by hang on 25-6-27.
//

#ifndef APP_DEFS_HPP
#define APP_DEFS_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dshot.hpp"
#include "custom_msgs/msg/write_lk_motor_openloop_control.hpp"
#include "custom_msgs/msg/write_lk_motor_speed_control_with_torque_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_torque_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control.hpp"
#include "custom_msgs/msg/write_lk_motor_single_round_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/write_on_board_pwm.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_dm_motor_speed_control.hpp"

namespace aim::ecat {
    class SlaveDevice;
}

namespace aim::ecat::task {
    constexpr uint8_t UNKNOWN_APP_ID = 250;
    constexpr uint8_t DJIRC_APP_ID = 1;
    constexpr uint8_t LK_APP_ID = 2;
    constexpr uint8_t HIPNUC_IMU_CAN_APP_ID = 3;
    constexpr uint8_t DSHOT_APP_ID = 4;
    constexpr uint8_t DJICAN_APP_ID = 5;
    constexpr uint8_t ONBOARD_PWM_APP_ID = 6;
    constexpr uint8_t EXTERNAL_PWM_APP_ID = 7;
    constexpr uint8_t MS5837_30BA_APP_ID = 8;
    constexpr uint8_t ADC_APP_ID = 9;
    constexpr uint8_t CAN_PMU_APP_ID = 10;
    constexpr uint8_t SBUS_RC_APP_ID = 11;
    constexpr uint8_t DM_MOTOR_APP_ID = 12;

    class TaskWrapper {
        uint8_t type_id_ = 0;
        std::string type_name_{"unknown_task"};
        bool has_publishers_{false};
        bool has_subscribers_{false};

    protected:
        std::shared_ptr<SlaveDevice> slave_device_;
        uint16_t pdowrite_offset_ = 0;
        uint16_t pdoread_offset_ = 0;
        uint8_t connection_lost_read_action_ = 0;
        uint8_t connection_lost_write_action_ = 0;
        inline static int shared_offset_ = 0;

        void load_slave_info(uint16_t slave_id, const std::string &prefix);

    public:
        explicit TaskWrapper(const uint8_t type_id, const std::string &type_name,
                             const bool has_publishers, const bool has_subscribers) {
            type_id_ = type_id;
            type_name_ = type_name;
            has_publishers_ = has_publishers;
            has_subscribers_ = has_subscribers;
        }

        virtual ~TaskWrapper() = default;

        virtual void init_sdo(uint8_t */* buf */, int */* offset */,
                              uint16_t /* slave_id */, const std::string &prefix);

        virtual void init_value();

        virtual void read();

        virtual void publish_empty_message();

        virtual void cleanup();

        [[nodiscard]] bool has_publishers() const {
            return has_publishers_;
        }

        [[nodiscard]] bool has_subscribers() const {
            return has_subscribers_;
        }

        [[nodiscard]] uint8_t get_type_id() const {
            return type_id_;
        }

        [[nodiscard]] std::string get_type_name() const {
            return type_name_;
        }

        [[nodiscard]] uint8_t get_connection_lost_read_action() const {
            return connection_lost_read_action_;
        }

        [[nodiscard]] uint8_t get_connection_lost_write_action() const {
            return connection_lost_write_action_;
        }
    };

    namespace dbus_rc {
        class DBUS_RC final : public TaskWrapper {
            static custom_msgs::msg::ReadDJIRC custom_msgs_readdjirc_shared_msg;
            rclcpp::Publisher<custom_msgs::msg::ReadDJIRC>::SharedPtr publisher_{};

        public:
            DBUS_RC() : TaskWrapper(DJIRC_APP_ID, "DJIRC", true, false) {
            }

            void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void read() override;

            void cleanup() override {
                if (publisher_) {
                    publisher_.reset();
                }
            }
        };
    }

    namespace hipnuc_imu {
        class HIPNUC_IMU_CAN final : public TaskWrapper {
            static sensor_msgs::msg::Imu sensor_msgs_imu_shared_msg;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_{};

            std::string conf_frame_name_{};

        public:
            HIPNUC_IMU_CAN() : TaskWrapper(HIPNUC_IMU_CAN_APP_ID, "HIPNUC_IMU", true, false) {
            }

            void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void read() override;
        };
    }

    namespace dji_motor {
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_CURRENT = 0x01;
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_SPEED = 0x02;
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_SINGLE_ROUND_POSITION = 0x03;

        class DJI_MOTOR final : public TaskWrapper {
            static custom_msgs::msg::ReadDJIMotor custom_msgs_readdjimotor_shared_msg;
            rclcpp::Publisher<custom_msgs::msg::ReadDJIMotor>::SharedPtr publisher_{};
            rclcpp::Subscription<custom_msgs::msg::WriteDJIMotor>::SharedPtr subscriber_{};

            bool is_motor_enabled[4]{false, false, false, false};

            void on_command(custom_msgs::msg::WriteDJIMotor::SharedPtr msg) const;

        public:
            DJI_MOTOR() : TaskWrapper(DJICAN_APP_ID, "DJI_MOTOR", true, true) {
            }

            void cleanup() override {
                if (publisher_) {
                    publisher_.reset();
                }

                if (subscriber_) {
                    subscriber_.reset();
                }
            }

            void init_sdo(uint8_t *buf, int *offset, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void init_value() override;

            void read() override;
        };
    }

    namespace pwm {
        class ONBOARD_PWM final : public TaskWrapper {
            rclcpp::Subscription<custom_msgs::msg::WriteOnBoardPWM>::SharedPtr subscriber_{};

            uint16_t init_value_ = 0;

            void on_command(custom_msgs::msg::WriteOnBoardPWM::SharedPtr msg) const;

        public:
            ONBOARD_PWM() : TaskWrapper(ONBOARD_PWM_APP_ID, "ONBOARD_PWM", false, true) {
            }

            void cleanup() override {
                if (subscriber_) {
                    subscriber_.reset();
                }
            }

            void init_sdo(uint8_t *buf, int *offset, uint16_t slave_id, const std::string &prefix) override;

            void init_value() override;
        };

        // TODO: task waiting for test
        // struct EXTERNAL_PWM {
        //     static constexpr uint8_t type_id = EXTERNAL_PWM_APP_ID;
        //     static constexpr auto type_enum = "EXTERNAL_PWM";
        //
        //     static void
        //     init_sdo(uint8_t *buf, int *offset, const uint32_t & /*sn*/, const uint16_t slave_id,
        //              const std::string &prefix) {
        //         memcpy(buf + *offset,
        //                sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
        //                                   {"uart_id", "pwm_period", "channel_num", "init_value"}),
        //                6);
        //         *offset += 6;
        //         node->create_and_insert_subscriber<custom_msgs::msg::WriteExternalPWM>(prefix, slave_id);
        //     }
        //
        //     static void
        //     init_value(uint8_t *buf, int *offset, const std::string &prefix) {
        //         const uint16_t init_value = get_field_as<uint16_t>(fmt::format("{}sdowrite_init_value", prefix));
        //         const uint8_t channel_num = get_field_as<uint8_t>(fmt::format("{}sdowrite_channel_num", prefix));
        //
        //         for (int i = 1; i <= channel_num; i++) {
        //             RCLCPP_INFO(cfg_logger, "prefix=%s will write init value=%d at m2s buf idx=%d", prefix.c_str(), init_value,
        //                         *offset);
        //             write_uint16(init_value, buf, offset);
        //         }
        //     }
        // };

        class DSHOT final : public TaskWrapper {
            rclcpp::Subscription<custom_msgs::msg::WriteDSHOT>::SharedPtr subscriber_{};

            uint16_t init_value_ = 0;

            void on_command(custom_msgs::msg::WriteDSHOT::SharedPtr msg) const;

        public:
            DSHOT() : TaskWrapper(DSHOT_APP_ID, "DSHOT", false, true) {
            }

            void cleanup() override {
                if (subscriber_) {
                    subscriber_.reset();
                }
            }

            void init_sdo(uint8_t *buf, int *offset, uint16_t slave_id, const std::string &prefix) override;

            void init_value() override;
        };
    }

    namespace ms5837 {
        constexpr uint8_t MS5837_30BA_OSR_256 = 0x01;
        constexpr uint8_t MS5837_30BA_OSR_512 = 0x02;
        constexpr uint8_t MS5837_30BA_OSR_1024 = 0x03;
        constexpr uint8_t MS5837_30BA_OSR_2048 = 0x04;
        constexpr uint8_t MS5837_30BA_OSR_4096 = 0x05;
        constexpr uint8_t MS5837_30BA_OSR_8192 = 0x06;

        // TODO: task waiting for test
        // struct MS5837_30BA {
        //     static constexpr uint8_t type_id = MS5837_30BA_APP_ID;
        //     static constexpr auto type_enum = "MS5837_30BA";
        //
        //     static void
        //     init_sdo(uint8_t *buf, int *offset, const uint32_t & /*sn*/, const uint8_t /*slave_id*/,
        //              const std::string &prefix) {
        //         memcpy(buf + *offset,
        //                sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
        //                                   {"i2c_id", "osr_id"}),
        //                2);
        //         *offset += 2;
        //         node->create_and_insert_publisher<custom_msgs::msg::ReadMS5837BA30>(prefix);
        //     }
        //
        //     static void
        //     read(const uint8_t *buf, int *offset, const std::string &prefix) {
        //         custom_msgs_readms5837ba30_shared_msg.header.stamp = rclcpp::Clock().now();
        //
        //         custom_msgs_readms5837ba30_shared_msg.temperature = read_int32(buf, offset) / 100.;
        //         custom_msgs_readms5837ba30_shared_msg.pressure = read_int32(buf, offset) / 10.;
        //
        //         EthercatNode::publish_msg<custom_msgs::msg::ReadMS5837BA30>(prefix, custom_msgs_readms5837ba30_shared_msg);
        //     }
        // };
    }

    namespace lk_motor {
        constexpr uint8_t LK_CTRL_TYPE_OPENLOOP_CURRENT = 0x01;
        constexpr uint8_t LK_CTRL_TYPE_TORQUE = 0x02;
        constexpr uint8_t LK_CTRL_TYPE_SPEED_WITH_TORQUE_LIMIT = 0x03;
        constexpr uint8_t LK_CTRL_TYPE_MULTI_ROUND_POSITION = 0x04;
        constexpr uint8_t LK_CTRL_TYPE_MULTI_ROUND_POSITION_WITH_SPEED_LIMIT = 0x05;
        constexpr uint8_t LK_CTRL_TYPE_SINGLE_ROUND_POSITION = 0x06;
        constexpr uint8_t LK_CTRL_TYPE_SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT = 0x07;

        class LK_MOTOR final : public TaskWrapper {
            static custom_msgs::msg::ReadLkMotor custom_msgs_readlkmotor_shared_msg;

            rclcpp::Publisher<custom_msgs::msg::ReadLkMotor>::SharedPtr publisher_{};

            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorOpenloopControl>::SharedPtr subscriber_openloop_control_
                    {};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorTorqueControl>::SharedPtr subscriber_torque_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>::SharedPtr
            subscriber_speed_with_torque_limit_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorMultiRoundPositionControl>::SharedPtr
            subscriber_multi_round_pos_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>::SharedPtr
            subscriber_multi_round_pos_with_speed_limit_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSingleRoundPositionControl>::SharedPtr
            subscriber_single_round_pos_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit>::SharedPtr
            subscriber_single_round_pos_with_speed_limit_control_{};

            void on_command_openloop_control(custom_msgs::msg::WriteLkMotorOpenloopControl::SharedPtr msg) const;

            void on_command_torque_control(custom_msgs::msg::WriteLkMotorTorqueControl::SharedPtr msg) const;

            void on_command_speed_with_torque_limit_control(
                custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit::SharedPtr msg) const;

            void on_command_multi_round_pos_control(
                custom_msgs::msg::WriteLkMotorMultiRoundPositionControl::SharedPtr msg) const;

            void on_command_multi_round_pos_with_speed_limit_control(
                custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit::SharedPtr msg) const;

            void on_command_single_round_pos_control(
                custom_msgs::msg::WriteLkMotorSingleRoundPositionControl::SharedPtr msg) const;

            void on_command_single_round_pos_with_speed_limit_control(
                custom_msgs::msg::WriteLkMotorSingleRoundPositionControlWithSpeedLimit::SharedPtr msg) const;

        public:
            LK_MOTOR() : TaskWrapper(LK_APP_ID, "LK_MOTOR", true, true) {
            }

            void cleanup() override {
                if (publisher_) {
                    publisher_.reset();
                }
                if (subscriber_openloop_control_) {
                    subscriber_openloop_control_.reset();
                }
                if (subscriber_torque_control_) {
                    subscriber_torque_control_.reset();
                }
                if (subscriber_speed_with_torque_limit_control_) {
                    subscriber_speed_with_torque_limit_control_.reset();
                }
                if (subscriber_multi_round_pos_control_) {
                    subscriber_multi_round_pos_control_.reset();
                }
                if (subscriber_multi_round_pos_with_speed_limit_control_) {
                    subscriber_multi_round_pos_with_speed_limit_control_.reset();
                }
                if (subscriber_single_round_pos_control_) {
                    subscriber_single_round_pos_control_.reset();
                }
                if (subscriber_single_round_pos_with_speed_limit_control_) {
                    subscriber_single_round_pos_with_speed_limit_control_.reset();
                }
            }

            void init_sdo(uint8_t *buf, int *offset, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void read() override;

            void init_value() override;
        };
    }

    namespace adc {
        // TODO: task waiting for test
        // struct ADC {
        //     static constexpr uint8_t type_id = ADC_APP_ID;
        //     static constexpr auto type_enum = "ADC";
        //
        //     static void
        //     init_sdo(uint8_t *buf, int *offset, const uint32_t & /*sn*/, const uint8_t /*slave_id*/,
        //              const std::string &prefix) {
        //         auto [sdo_buf, sdo_len] = sdo_data.build_buf(fmt::format("{}sdowrite_", prefix),
        //                                                      {
        //                                                          "channel1_coefficient_per_volt",
        //                                                          "channel2_coefficient_per_volt"
        //                                                      });
        //         memcpy(buf + *offset, sdo_buf, sdo_len);
        //         *offset += sdo_len;
        //         node->create_and_insert_publisher<custom_msgs::msg::ReadADC>(prefix);
        //     }
        //
        //     static void
        //     read(const uint8_t *buf, int *offset, const std::string &prefix) {
        //         custom_msgs_readadc_shared_msg.header.stamp = rclcpp::Clock().now();
        //
        //         custom_msgs_readadc_shared_msg.channel1 = read_float(buf, offset);
        //         custom_msgs_readadc_shared_msg.channel2 = read_float(buf, offset);
        //
        //         EthercatNode::publish_msg<custom_msgs::msg::ReadADC>(prefix, custom_msgs_readadc_shared_msg);
        //     }
        // };
    }

    namespace pmu_uavcan {
        // TODO: task waiting for test
        // struct CAN_PMU {
        //     static constexpr uint8_t type_id = CAN_PMU_APP_ID;
        //     static constexpr auto type_enum = "CAN_PMU";
        //
        //     static void
        //     init_sdo(uint8_t * /*buf*/, int * /*offset*/, const uint32_t & /*sn*/, const uint8_t /*slave_id*/,
        //              const std::string &prefix) {
        //         node->create_and_insert_publisher<custom_msgs::msg::ReadCANPMU>(prefix);
        //     }
        //
        //     static void
        //     read(const uint8_t *buf, int *offset, const std::string &prefix) {
        //         custom_msgs_readcanpmu_shared_msg.header.stamp = rclcpp::Clock().now();
        //
        //         custom_msgs_readcanpmu_shared_msg.temperature = read_float16(buf, offset) - 273.15f;
        //         custom_msgs_readcanpmu_shared_msg.voltage = read_float16(buf, offset);
        //         custom_msgs_readcanpmu_shared_msg.current = read_float16(buf, offset);
        //
        //         EthercatNode::publish_msg<custom_msgs::msg::ReadCANPMU>(prefix, custom_msgs_readcanpmu_shared_msg);
        //     }
        // };
    }

    namespace sbus_rc {
        class SBUS_RC final : public TaskWrapper {
            static custom_msgs::msg::ReadSBUSRC custom_msgs_readsbusrc_shared_msg;
            rclcpp::Publisher<custom_msgs::msg::ReadSBUSRC>::SharedPtr publisher_{};

        public:
            SBUS_RC() : TaskWrapper(SBUS_RC_APP_ID, "SBUSRC", true, false) {
            }

            void cleanup() override {
                if (publisher_) {
                    publisher_.reset();
                }
            }

            void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void read() override;
        };
    }

    namespace dm_motor {
        constexpr uint8_t DM_CTRL_TYPE_MIT = 0x01;
        constexpr uint8_t DM_CTRL_TYPE_POSITION_WITH_SPEED_LIMIT = 0x02;
        constexpr uint8_t DM_CTRL_TYPE_SPEED = 0x03;

        class DM_MOTOR final : public TaskWrapper {
            static custom_msgs::msg::ReadDmMotor custom_msgs_readdmmotor_shared_msg;

            rclcpp::Publisher<custom_msgs::msg::ReadDmMotor>::SharedPtr publisher_{};

            rclcpp::Subscription<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr subscriber_mit_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr
            subscriber_pos_with_speed_limit_control_{};
            rclcpp::Subscription<custom_msgs::msg::WriteDmMotorSpeedControl>::SharedPtr subscriber_speed_control_{};

            void on_command_mit_control(custom_msgs::msg::WriteDmMotorMITControl::SharedPtr msg) const;

            void on_command_pos_with_speed_limit_control(
                custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit::SharedPtr msg) const;

            void on_command_speed_control(custom_msgs::msg::WriteDmMotorSpeedControl::SharedPtr msg) const;

            float pmax_ = 0.0f;
            float vmax_ = 0.0f;
            float tmax_ = 0.0f;

        public:
            DM_MOTOR() : TaskWrapper(DM_MOTOR_APP_ID, "DM_MOTOR", true, true) {
            }

            void cleanup() override {
                if (publisher_) {
                    publisher_.reset();
                }
                if (subscriber_mit_control_) {
                    subscriber_mit_control_.reset();
                }
                if (subscriber_pos_with_speed_limit_control_) {
                    subscriber_pos_with_speed_limit_control_.reset();
                }
                if (subscriber_speed_control_) {
                    subscriber_speed_control_.reset();
                }
            }

            void init_sdo(uint8_t *buf, int *offset, uint16_t slave_id, const std::string &prefix) override;

            void publish_empty_message() override;

            void read() override;

            void init_value() override;
        };
    }
}

#endif //APP_DEFS_HPP
