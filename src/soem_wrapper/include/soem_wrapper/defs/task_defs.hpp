//
// Created by hang on 25-6-27.
//

#ifndef APP_DEFS_HPP
#define APP_DEFS_HPP

#include "soem_wrapper/utils/logger_utils.hpp"

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_djirc.hpp"
#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"

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
    public:
        virtual ~TaskWrapper() = default;

        virtual uint8_t get_type_id() {
            return 0;
        }

        virtual std::string get_type_name() {
            return "unknown_task";
        }

        virtual void init_sdo(uint8_t */* buf */, int */* offset */, const uint32_t /* sn */,
                              const uint8_t /* slave_id */,
                              const std::string &prefix) {
            RCLCPP_ERROR(*logging::get_data_logger(),
                         "init_sdo was not overridden and got called for the task: %s, prefix: %s",
                         get_type_name().c_str(),
                         prefix.c_str());
        }

        virtual void init_value(uint8_t */* buf */, int */* offset */, const std::string &prefix) {
            RCLCPP_ERROR(*logging::get_data_logger(),
                         "init_value was not overridden and got called for the task: %s, prefix: %s",
                         get_type_name().c_str(),
                         prefix.c_str());
        }

        virtual void read(const uint8_t */* buf */, int */* offset */, const std::string &prefix) {
            RCLCPP_ERROR(*logging::get_data_logger(),
                         "read was not overridden and got called for the task: %s, prefix: %s",
                         get_type_name().c_str(),
                         prefix.c_str());
        }

        virtual void publish_empty_message(const std::string &prefix) {
            RCLCPP_ERROR(*logging::get_data_logger(),
                         "publish_empty_message was not overridden and got called for the task: %s, prefix: %s",
                         get_type_name().c_str(),
                         prefix.c_str());
        }
    };

    namespace dbus_rc {

        class DBUS_RC final : public TaskWrapper {
            static custom_msgs::msg::ReadDJIRC custom_msgs_readdjirc_shared_msg;

            DBUS_RC() = default;

        public:
            DBUS_RC(const DBUS_RC &) = delete;

            DBUS_RC &operator=(const DBUS_RC &) = delete;

            static DBUS_RC &instance() {
                static DBUS_RC inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return DJIRC_APP_ID;
            }

            std::string get_type_name() override {
                return "DJIRC";
            }

            void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint32_t /*sn*/, uint8_t /*slave_id*/,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;
        };
    }

    namespace hipnuc_imu {
        class HIPNUC_IMU_CAN final : public TaskWrapper {
            static sensor_msgs::msg::Imu sensor_msgs_imu_shared_msg;

            HIPNUC_IMU_CAN() = default;

        public:
            HIPNUC_IMU_CAN(const HIPNUC_IMU_CAN &) = delete;

            HIPNUC_IMU_CAN &operator=(const HIPNUC_IMU_CAN &) = delete;

            static HIPNUC_IMU_CAN &instance() {
                static HIPNUC_IMU_CAN inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return HIPNUC_IMU_CAN_APP_ID;
            }

            std::string get_type_name() override {
                return "HIPNUC_IMU";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t /*slave_id*/,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;
        };
    }

    namespace dji_motor {
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_CURRENT = 0x01;
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_SPEED = 0x02;
        constexpr uint8_t DJIMOTOR_CTRL_TYPE_SINGLE_ROUND_POSITION = 0x03;

        class DJI_MOTOR final : public TaskWrapper {
            static custom_msgs::msg::ReadDJIMotor custom_msgs_readdjimotor_shared_msg;

            DJI_MOTOR() = default;

        public:
            DJI_MOTOR(const DJI_MOTOR &) = delete;

            DJI_MOTOR &operator=(const DJI_MOTOR &) = delete;

            static DJI_MOTOR &instance() {
                static DJI_MOTOR inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return DJICAN_APP_ID;
            }

            std::string get_type_name() override {
                return "DJI_MOTOR";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t slave_id,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;
        };
    }

    namespace dshot {
        class DSHOT final : public TaskWrapper {
            DSHOT() = default;

        public:
            DSHOT(const DSHOT &) = delete;

            DSHOT &operator=(const DSHOT &) = delete;

            static DSHOT &instance() {
                static DSHOT inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return DSHOT_APP_ID;
            }

            std::string get_type_name() override {
                return "DSHOT";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t slave_id,
                          const std::string &prefix) override;

            void init_value(uint8_t *buf, int *offset, const std::string &prefix) override;
        };
    }

    namespace pwm {
        class ONBOARD_PWM final : public TaskWrapper {
            ONBOARD_PWM() = default;

        public:
            ONBOARD_PWM(const ONBOARD_PWM &) = delete;

            ONBOARD_PWM &operator=(const ONBOARD_PWM &) = delete;

            static ONBOARD_PWM &instance() {
                static ONBOARD_PWM inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return ONBOARD_PWM_APP_ID;
            }

            std::string get_type_name() override {
                return "ONBOARD_PWM";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t slave_id,
                          const std::string &prefix) override;

            void init_value(uint8_t *buf, int *offset, const std::string &prefix) override;
        };

        // TODO: task waiting for test
        // struct EXTERNAL_PWM {
        //     static constexpr uint8_t type_id = EXTERNAL_PWM_APP_ID;
        //     static constexpr auto type_enum = "EXTERNAL_PWM";
        //
        //     static void
        //     init_sdo(uint8_t *buf, int *offset, const uint32_t & /*sn*/, const uint8_t slave_id,
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

            LK_MOTOR() = default;

        public:
            LK_MOTOR(const LK_MOTOR &) = delete;

            LK_MOTOR &operator=(const LK_MOTOR &) = delete;

            static LK_MOTOR &instance() {
                static LK_MOTOR inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return LK_APP_ID;
            }

            std::string get_type_name() override {
                return "LK_MOTOR";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t slave_id,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;

            void init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) override;
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

            SBUS_RC() = default;

        public:
            SBUS_RC(const SBUS_RC &) = delete;

            SBUS_RC &operator=(const SBUS_RC &) = delete;

            static SBUS_RC &instance() {
                static SBUS_RC inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return SBUS_RC_APP_ID;
            }

            std::string get_type_name() override {
                return "SBUSRC";
            }

            void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint32_t /*sn*/, uint8_t /*slave_id*/,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;
        };
    }

    namespace dm_motor {
        constexpr uint8_t DM_CTRL_TYPE_MIT = 0x01;
        constexpr uint8_t DM_CTRL_TYPE_POSITION_WITH_SPEED_LIMIT = 0x02;
        constexpr uint8_t DM_CTRL_TYPE_SPEED = 0x03;

        class DM_MOTOR final : public TaskWrapper {
            static custom_msgs::msg::ReadDmMotor custom_msgs_readdmmotor_shared_msg;

            DM_MOTOR() = default;

        public:
            DM_MOTOR(const DM_MOTOR &) = delete;

            DM_MOTOR &operator=(const DM_MOTOR &) = delete;

            static DM_MOTOR &instance() {
                static DM_MOTOR inst;
                return inst;
            }

            uint8_t get_type_id() override {
                return DM_MOTOR_APP_ID;
            }

            std::string get_type_name() override {
                return "DM_MOTOR";
            }

            void init_sdo(uint8_t *buf, int *offset, uint32_t /*sn*/, uint8_t slave_id,
                          const std::string &prefix) override;

            void publish_empty_message(const std::string &prefix) override;

            void read(const uint8_t *buf, int *offset, const std::string &prefix) override;

            void init_value(uint8_t *buf, int *offset, const std::string &/* prefix */) override;
        };
    }
}

#endif //APP_DEFS_HPP
