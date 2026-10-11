//
// Created by hang on 10/10/26.
//

#ifndef SOEM_WRAPPER_DEVICE_SLAVE_DEVICE_HPP
#define SOEM_WRAPPER_DEVICE_SLAVE_DEVICE_HPP

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "soem_wrapper/utils/pi_mutex.hpp"

namespace aim::utils::config {
    class ConfigurationParser;
}

namespace aim::ecat {
    class SdoWorker;
}

namespace aim::ecat::device {
    struct DeviceInfo {
        uint16_t index;
        uint32_t vendor_id;
        uint32_t product_code;
        std::string name;
    };

    /**
     * passed to on_configure, called inside the PRE-OP -> SAFE-OP hook
     */
    struct ConfigContext {
        bool is_reconfigure;
        utils::config::ConfigurationParser &config;
    };

    /**
     * passed to on_setup, called once from the main thread after the io map is configured
     */
    struct SetupContext {
        rclcpp::Node &node;
        utils::config::ConfigurationParser &config;
        // global worker for blocking sdo operations at runtime, started after all slaves reached OP
        SdoWorker &sdo_worker;
    };

    /**
     * passed to on_cycle, called from the realtime data thread with mtx_ held
     */
    struct CycleContext {
        const rclcpp::Time &now;
    };

    class SlaveDevice {
    public:
        explicit SlaveDevice(DeviceInfo info);

        virtual ~SlaveDevice() = default;

        SlaveDevice(const SlaveDevice &) = delete;

        SlaveDevice &operator=(const SlaveDevice &) = delete;

        /**
         * PO2SO hook entry, calls on_configure
         * @return 1 on success
         */
        int configure();

        /**
         * allocate rx/tx buffers according to the mapped process data size
         * must be called after ec_config_map
         */
        void post_map();

        /**
         * calls on_setup and marks the device as configured on success,
         * an exception thrown by on_setup (e.g. a missing config key) is logged and treated as a failure
         */
        bool setup(SetupContext &ctx);

        virtual int on_configure(ConfigContext &ctx);

        virtual bool on_setup(SetupContext &ctx);

        virtual void on_op_reached();

        virtual void on_receive();

        virtual void on_cycle(const CycleContext &ctx);

        virtual void on_transmit();

        virtual void write_safe_outputs();

        [[nodiscard]] virtual bool is_configured() const;

        [[nodiscard]] virtual bool is_ready() const;

        virtual void pre_recover();

        virtual void reset_state();

        virtual void recover_state();

        virtual void on_connection_lost();

        [[nodiscard]] virtual bool is_recover_rejected() const;

        virtual void cleanup();

        [[nodiscard]] uint16_t get_index() const {
            return info_.index;
        }

        [[nodiscard]] uint32_t get_vendor_id() const {
            return info_.vendor_id;
        }

        [[nodiscard]] uint32_t get_product_code() const {
            return info_.product_code;
        }

        [[nodiscard]] const std::string &get_name() const {
            return info_.name;
        }

        [[nodiscard]] const std::string &get_config_prefix() const {
            return config_prefix_;
        }

        [[nodiscard]] uint32_t input_size() const;

        [[nodiscard]] uint32_t output_size() const;

        std::vector<uint8_t> &rx_buf() {
            return rx_buf_;
        }

        std::vector<uint8_t> &tx_buf() {
            return tx_buf_;
        }

        // shared by the realtime data thread and non-realtime threads (executor, health checker, sdo worker),
        // priority inheritance, so a non-realtime holder can't stall the realtime thread
        mutable utils::PiMutex mtx_;

    protected:
        /**
         * blocking SDO read, little endian
         */
        template<typename T>
        std::optional<T> sdo_read(const uint16_t index, const uint8_t sub_index) const {
            T value{};
            if (!sdo_read_raw(index, sub_index, &value, sizeof(T))) {
                return std::nullopt;
            }
            return value;
        }

        /**
         * blocking SDO write, little endian
         */
        template<typename T>
        bool sdo_write(const uint16_t index, const uint8_t sub_index, T value) const {
            return sdo_write_raw(index, sub_index, &value, sizeof(T));
        }

        void set_config_prefix(std::string prefix) {
            config_prefix_ = std::move(prefix);
        }

        [[nodiscard]] bool is_operational() const;

        bool ready_{false};

    private:
        bool sdo_read_raw(uint16_t index, uint8_t sub_index, void *buf, int size) const;

        bool sdo_write_raw(uint16_t index, uint8_t sub_index, void *buf, int size) const;

        DeviceInfo info_;
        std::string config_prefix_{};

        std::vector<uint8_t> rx_buf_{};
        std::vector<uint8_t> tx_buf_{};

        bool configured_once_{false};
        bool setup_done_{false};
    };
}

#endif //SOEM_WRAPPER_DEVICE_SLAVE_DEVICE_HPP
