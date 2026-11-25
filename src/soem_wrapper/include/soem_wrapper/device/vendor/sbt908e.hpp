//
// Created by hang on 10/10/26.
//

#ifndef SOEM_WRAPPER_DEVICE_VENDOR_SBT908E_HPP
#define SOEM_WRAPPER_DEVICE_VENDOR_SBT908E_HPP

#include <array>
#include <atomic>
#include <deque>
#include <functional>

#include "soem_wrapper/device/slave_device.hpp"
#include "soem_wrapper/sdo_worker.hpp"
#include "custom_msgs/msg/read_sbt908_e.hpp"
#include "custom_msgs/srv/sbt908_e_command.hpp"
#include "custom_msgs/srv/sbt908_e_gain_calibration.hpp"
#include "custom_msgs/srv/sbt908_e_sensitivity_calibration.hpp"
#include "custom_msgs/srv/sbt908_e_set_calibration_parameters.hpp"

namespace aim::ecat::device::vendor {
    constexpr uint32_t SBT908E_VENDOR_ID = 0x00000FFB;
    constexpr uint32_t SBT908E_PRODUCT_CODE = 0x00000001;

    /**
     * SBT908E digital loadcell amplifier
     * publishes the gross force and the peak value of the configured channels at most 2000 times per second,
     * provides services to zero / clear peak through the pdo command,
     * and to set the calibration parameters / calibrate the zero point / the gain / the sensitivity through the
     * sdo command (sdo only commands, manual 9.6.1 - 9.6.4),
     * the service is answered after the execution result is reported by the slave
     *
     * config keys (prefix slave{index}_):
     *  - pub_topic:           !std::string, required
     *  - channel_count:       !uint8_t, required, 1-20, number of channels of the device
     *
     * precision parameters, required, written into all channels on every startup:
     *  - ad_speed:            !uint8_t,  0 = 10, 1 = 40, 2 = 640, 3 = 1280 conversions per second
     *  - ad_resolution:       !uint8_t,  16-24 bits
     *  - filter_type:         !uint8_t,  0-10
     *  - filter1_level:       !uint8_t,  0-50
     *  - filter1_count:       !uint8_t,  0-9
     *  - filter2_level:       !uint8_t,  0-9
     *  - filter_stable:       !uint8_t,  0 = off, 1 = on
     *
     * the unit, the division and the calibration are not touched on startup, they are set by the services,
     * the division is read from the slave to convert the values
     */
    class SBT908E final : public SlaveDevice {
    public:
        explicit SBT908E(const DeviceInfo &info);

        int on_configure(ConfigContext &ctx) override;

        bool on_setup(SetupContext &ctx) override;

        void on_cycle(const CycleContext &ctx) override;

        void cleanup() override;

    private:
        using CommandSrv = custom_msgs::srv::SBT908ECommand;
        using GainCalibrationSrv = custom_msgs::srv::SBT908EGainCalibration;
        using SetCalibrationParametersSrv = custom_msgs::srv::SBT908ESetCalibrationParameters;
        using SensitivityCalibrationSrv = custom_msgs::srv::SBT908ESensitivityCalibration;
        // sends the response of a service request, holds the service and the request header
        using Responder = std::function<void(bool success, const std::string &message)>;

        // default pdo mapping, SM2 = 0x1600, SM3 = 0x1A00
        static constexpr uint32_t INPUT_SIZE = 182;
        static constexpr uint32_t OUTPUT_SIZE = 24;
        // the pdo carries 20 channels, the number of channels of the device is configured
        static constexpr int MAX_CHANNEL_COUNT = 20;
        static constexpr uint8_t ALL_CHANNELS = 255;

        // inputs
        // 0x6000:01-14 CHn_Force_Gross, INT32
        static constexpr int FORCE_GROSS_OFFSET = 0;
        // 0x6001:01 DataType, UINT16, type of the general data
        static constexpr int GENERAL_DATA_TYPE_OFFSET = 80;
        // 0x6001:02-15 CHn_DataValue, INT32
        static constexpr int GENERAL_DATA_VALUE_OFFSET = 82;
        static constexpr uint16_t GENERAL_DATA_TYPE_PEAK = 0;
        // 0x6003:01-03 CommandReturn Return1-3, UINT32, execution result of the pdo command,
        // 4 bits per channel, Return1 bit 3-0 = channel 1, bit 7-4 = channel 2, ...
        // not described by the manual, the format is the same as the Return1-3 of the sdo command (0x8000:06-08)
        static constexpr int COMMAND_RETURN_OFFSET = 170;
        static constexpr uint8_t COMMAND_RESULT_NONE = 0b0000;
        static constexpr uint8_t COMMAND_RESULT_ERROR = 0b0001;
        static constexpr uint8_t COMMAND_RESULT_OK = 0b0010;
        // not reported by the slave, the result can't be read through the sdo
        static constexpr uint8_t COMMAND_RESULT_READ_FAILED = 0xFF;

        // outputs, 0x7000 PDO_Command
        static constexpr int COMMAND_CHANNEL_OFFSET = 0;
        static constexpr int COMMAND_CODE_OFFSET = 16;
        static constexpr uint32_t COMMAND_NONE = 0;
        static constexpr uint32_t COMMAND_ZERO = 1;
        static constexpr uint32_t COMMAND_CLEAR_PEAK = 3;
        static constexpr uint32_t COMMAND_CLEAR_FLAGS = 30;
        // timeout of each phase (clearing the execution flags / waiting for the execution result)
        static constexpr int64_t COMMAND_TIMEOUT_NS = 500'000'000;

        // min interval between two messages, 2 kHz, the data cycle is free running and much faster
        static constexpr int64_t PUBLISH_INTERVAL_NS = 500'000;

        // 0x8000 SDO_Command, executed every time the code (sub 0x05) is written, the result is reported in
        // Return1-3 (sub 0x06-0x08), same format as 0x6003
        static constexpr uint16_t SDO_COMMAND_INDEX = 0x8000;
        static constexpr uint8_t SDO_COMMAND_CHANNEL_SUB_INDEX = 0x01;
        static constexpr uint8_t SDO_COMMAND_PARA1_SUB_INDEX = 0x02;
        static constexpr uint8_t SDO_COMMAND_PARA2_SUB_INDEX = 0x03;
        static constexpr uint8_t SDO_COMMAND_PARA3_SUB_INDEX = 0x04;
        static constexpr uint8_t SDO_COMMAND_CODE_SUB_INDEX = 0x05;
        static constexpr uint8_t SDO_COMMAND_RETURN1_SUB_INDEX = 0x06;
        // set calibration parameters (manual 9.6.1), Para1 = unit, Para2 = division index,
        // Para3 = max weighing value as an integer with the decimal places of the division
        static constexpr uint32_t COMMAND_SET_CALIBRATION = 50;
        static constexpr int32_t MAX_FULL_RANGE = 8'000'000;
        // sensitivity calibration (manual 9.6.4), Para1 = sensitivity in 0.0001 mV/V,
        // Para2 = range of a single sensor as an integer with the decimal places of the division
        static constexpr uint32_t COMMAND_SENSITIVITY_CALIBRATION = 53;
        static constexpr int32_t MIN_SENSITIVITY = 1'000;
        static constexpr int32_t MAX_SENSITIVITY = 78'000;
        // zero calibration with the current load (manual 9.6.2), no parameter
        static constexpr uint32_t COMMAND_ZERO_CALIBRATION = 51;
        // gain calibration with a known weight (manual 9.6.3),
        // Para1 = weight as an integer with the decimal places of the division
        static constexpr uint32_t COMMAND_GAIN_CALIBRATION = 52;
        static constexpr int32_t MAX_CALIBRATION_WEIGHT = 8'000'000;
        // timeout of each phase (clearing the execution flags / waiting for the execution result)
        static constexpr int SDO_COMMAND_TIMEOUT_MS = 1000;
        static constexpr int SDO_COMMAND_POLL_INTERVAL_MS = 10;

        // 0x8002 CommPara, sub 0x02 PDO_DataType, UINT8, type of the general data, saved in the slave
        static constexpr uint16_t COMM_PARA_INDEX = 0x8002;
        static constexpr uint8_t PDO_DATA_TYPE_SUB_INDEX = 0x02;

        // 0x8020 + (n - 1) BasePara_n, UINT8
        static constexpr uint16_t BASE_PARA_INDEX = 0x8020;
        static constexpr uint8_t BASE_UNIT_SUB_INDEX = 0x01;
        static constexpr uint8_t AD_SPEED_SUB_INDEX = 0x0E;
        static constexpr uint8_t FILTER_TYPE_SUB_INDEX = 0x0F;
        static constexpr uint8_t FILTER1_LEVEL_SUB_INDEX = 0x10;
        static constexpr uint8_t FILTER1_COUNT_SUB_INDEX = 0x11;
        static constexpr uint8_t FILTER2_LEVEL_SUB_INDEX = 0x12;
        static constexpr uint8_t FILTER_STABLE_SUB_INDEX = 0x13;
        static constexpr uint8_t AD_RESOLUTION_SUB_INDEX = 0x14;
        // 0x8120 + (n - 1) CaliPara_n, sub 0x02 Div, UINT8, index into DIVISIONS
        static constexpr uint16_t CALI_PARA_INDEX = 0x8120;
        // 0x8120 + (n - 1) CaliPara_n, sub 0x01 Unit, UINT8, the unit is stored in BasePara and CaliPara,
        // the one in BasePara is written after the calibration parameters are set to keep them consistent
        static constexpr uint8_t CALI_UNIT_SUB_INDEX = 0x01;
        static constexpr uint8_t DIV_SUB_INDEX = 0x02;

        // division table of the manual (9.6.1), the slave stores the index
        static constexpr std::array<double, 18> DIVISIONS = {
            0.0001, 0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05,
            0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50
        };
        // decimal places of the integer values (force, peak, calibration) for each division
        static constexpr std::array<int, 18> DIVISION_DECIMALS = {
            4, 4, 4, 3, 3, 3, 2, 2, 2,
            1, 1, 1, 0, 0, 0, 0, 0, 0
        };
        // unit table of the manual (9.6.1), the slave stores the index
        static constexpr std::array<const char *, 5> UNITS = {"none", "g", "kg", "t", "N"};
        // conversions per second, the slave stores the index, only used for the log
        static constexpr std::array<uint16_t, 4> AD_SPEEDS = {10, 40, 640, 1280};

        // esc register, sm watchdog time for process data, 0 = disabled
        static constexpr uint16_t ESC_REG_SM_WATCHDOG_TIME = 0x0420;
        // bit 6 of the sm control byte (low byte of SMflags), watchdog trigger enable
        static constexpr uint32_t SM_CONTROL_WATCHDOG_TRIGGER = 0x00000040;
        // 0x1C32 SM output parameter / 0x1C33 SM input parameter, sub 0x01 Synchronization Type, UINT16
        // default 0x0001 (SM2 synchron) / 0x0022 (synchron with SM2), switch both to free run
        static constexpr uint16_t SM_OUTPUT_PARAMETER_INDEX = 0x1C32;
        static constexpr uint16_t SM_INPUT_PARAMETER_INDEX = 0x1C33;
        static constexpr uint8_t SYNC_TYPE_SUB_INDEX = 0x01;
        static constexpr uint16_t SYNC_TYPE_FREE_RUN = 0x0000;

        /**
         * precision parameters, the values stored in the slave
         */
        struct PrecisionConfig {
            uint8_t ad_speed;
            uint8_t ad_resolution;
            uint8_t filter_type;
            uint8_t filter1_level;
            uint8_t filter1_count;
            uint8_t filter2_level;
            uint8_t filter_stable;
        };

        /**
         * a service request waiting to be executed through the pdo command
         */
        struct PendingCommand {
            Responder respond;
            uint8_t channel;
            uint32_t code;
        };

        /**
         * a service request to be executed through the sdo command by the global sdo worker
         */
        struct SdoRequest {
            Responder respond;
            uint8_t channel;
            uint32_t code;
            int32_t para1;
            int32_t para2;
            int32_t para3;
            const char *name;
        };

        /**
         * an executed (or rejected) command waiting for its response to be sent from the executor
         */
        struct FinishedCommand {
            Responder respond;
            bool success;
            std::string message;
        };

        template<typename SrvT>
        static Responder make_responder(std::shared_ptr<rclcpp::Service<SrvT> > service,
                                        std::shared_ptr<rmw_request_id_t> header) {
            return [service = std::move(service), header = std::move(header)](const bool success,
                                                                              const std::string &message) {
                typename SrvT::Response response;
                response.success = success;
                response.message = message;
                service->send_response(*header, response);
            };
        }

        /**
         * create a service whose response is deferred, the handler gets the responder to answer the request later
         */
        template<typename SrvT>
        void create_deferred_service(rclcpp::Node &node,
                                     const std::string &name,
                                     std::function<void(Responder, const typename SrvT::Request &)> handler) {
            // rclcpp picks the deferred response callback by the exact argument types of the callback,
            // so the std::function type is given explicitly, the lambda itself can take the request by reference
            std::function<void(std::shared_ptr<rclcpp::Service<SrvT> >,
                               std::shared_ptr<rmw_request_id_t>,
                               std::shared_ptr<typename SrvT::Request>)> callback =
                    [handler = std::move(handler)](std::shared_ptr<rclcpp::Service<SrvT> > service,
                                                   std::shared_ptr<rmw_request_id_t> header,
                                                   const std::shared_ptr<typename SrvT::Request> &request) {
                handler(make_responder(std::move(service), std::move(header)), *request);
            };
            services_.push_back(node.create_service<SrvT>(name, std::move(callback)));
        }

        enum class CommandPhase {
            IDLE,
            // command 30 is held in the outputs, waiting for the execution flags to be cleared
            CLEAR_FLAGS,
            // the command is held in the outputs, waiting for the execution result
            EXECUTE
        };

        bool set_free_run(uint16_t index) const;

        bool load_precision_config(const utils::config::ConfigurationParser &config, PrecisionConfig &out) const;

        bool apply_precision_config(const PrecisionConfig &config);

        /**
         * read the division of a channel (0-based) from the slave, update the decimal places and the scale
         */
        bool load_division(int ch, std::string *error = nullptr);

        /**
         * read the value, write it only if it differs (the parameters are saved into the flash of the slave),
         * then read back to make sure the slave accepted it
         */
        bool write_param(uint16_t index, uint8_t sub_index, uint8_t value, const std::string &name) const;

        /**
         * run a command through the sdo command object (0x8000) and wait for its execution result,
         * blocking, only used in on_setup and the jobs of the sdo worker, never while holding mtx_
         * @param channel 1-based channel, or ALL_CHANNELS
         * @param result_message the execution result of each channel, optional
         */
        bool run_sdo_command(uint8_t channel, uint32_t code, int32_t para1, int32_t para2, int32_t para3,
                             const std::string &name, std::string *result_message = nullptr) const;

        void create_command_service(rclcpp::Node &node, const std::string &name, uint32_t code);

        void create_sdo_command_services(rclcpp::Node &node, const std::string &srv_prefix);

        /**
         * answer the request with a failure, runs with mtx_ held
         */
        void fail_request(Responder &respond, std::string message);

        /**
         * answer the request with a failure if the channel is invalid or the slave is not operational,
         * runs with mtx_ held
         * @return true if the request is rejected
         */
        bool reject_request(Responder &respond, uint8_t channel);

        /**
         * submit a request to the global sdo worker, the result is put into finished_commands_ by the job,
         * runs in the executor with mtx_ held
         */
        void submit_sdo_request(SdoRequest request);

        /**
         * the unit in BasePara is written as well to keep it consistent, and the new division is loaded,
         * runs in the job of the sdo worker after the calibration parameters are set
         */
        bool after_set_calibration(uint8_t channel, uint8_t unit, std::string &message);


        /**
         * runs in the data thread with mtx_ held
         */
        void process_command(int64_t now_ns);

        void write_command(uint8_t channel, uint32_t code);

        /**
         * execution result of a channel (0-based) from the inputs, COMMAND_RESULT_*
         */
        uint8_t read_command_result(int ch);

        /**
         * e.g. "CH1=ok, CH2=error, CH3=not executed"
         */
        std::string describe_command_results(int first_ch, int last_ch);

        /**
         * reset the command code to 0, answer the running command, runs with mtx_ held
         */
        void finish_command(bool success, std::string message);

        /**
         * fail the running and all pending commands, runs with mtx_ held
         */
        void fail_all_commands(const std::string &reason);

        /**
         * runs in the executor, sends the responses of the finished commands
         */
        void send_command_responses();

        custom_msgs::msg::ReadSBT908E custom_msgs_readsbt908e_shared_msg{};
        rclcpp::Publisher<custom_msgs::msg::ReadSBT908E>::SharedPtr publisher_{};

        std::vector<rclcpp::ServiceBase::SharedPtr> services_{};
        rclcpp::TimerBase::SharedPtr response_timer_{};

        // guarded by mtx_
        std::deque<PendingCommand> pending_commands_{};
        std::deque<FinishedCommand> finished_commands_{};
        CommandPhase command_phase_ = CommandPhase::IDLE;
        int64_t command_phase_start_ns_ = 0;

        // global worker for the sdo only commands, set in on_setup
        SdoWorker *sdo_worker_ = nullptr;

        int channel_count_ = 0;
        // scale to convert the published values with the decimal places of the division of each channel,
        // updated by the sdo worker when the calibration parameters are set
        std::array<std::atomic<double>, MAX_CHANNEL_COUNT> scale_{};
        int64_t last_publish_ns_ = 0;
    };
}

#endif //SOEM_WRAPPER_DEVICE_VENDOR_SBT908E_HPP
