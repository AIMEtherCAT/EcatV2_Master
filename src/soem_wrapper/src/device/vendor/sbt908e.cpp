//
// Created by hang on 10/10/26.
//
#include "soem_wrapper/device/vendor/sbt908e.hpp"
#include "soem_wrapper/utils/config_utils.hpp"
#include "soem_wrapper/utils/io_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "ethercat.h"

namespace aim::ecat::device::vendor {
    using namespace logging;
    using namespace utils::config;
    using namespace io::little_endian;

    namespace {
        const char *command_result_text(const uint8_t result) {
            switch (result) {
                case 0b0000:
                    return "not executed";
                case 0b0001:
                    return "error";
                case 0b0010:
                    return "ok";
                case 0xFF:
                    return "read failed";
                default:
                    return "unknown";
            }
        }
    }

    SBT908E::SBT908E(const DeviceInfo &info) : SlaveDevice(info) {
    }

    int SBT908E::on_configure(ConfigContext &ctx) {
        // the slave enables the sm watchdog for the outputs (SM2) by default,
        // disable it so the slave won't drop to SAFE_OP + ERROR (AL status 0x001b)
        // when no process data is exchanged for a while.
        // called on every PRE-OP -> SAFE-OP transition, including reconfiguration after a recovery.
        // note: the outputs (Command, IO_Out) won't be reset by the slave on master loss anymore
        //
        // the watchdog time (0x0420) and the watchdog trigger bit in the output sm control byte must match,
        // otherwise the slave rejects SAFE_OP with AL status 0x001f (Invalid watchdog configuration).
        // this hook runs before soem writes the sm registers in ec_config_map,
        // ec_reconfig_slave writes the sm registers from the same flags, so clearing them once is enough
        for (int sm = 0; sm < EC_MAXSM; sm++) {
            if (ec_slave[get_index()].SMtype[sm] == 3) {
                ec_slave[get_index()].SM[sm].SMflags =
                        htoel(etohl(ec_slave[get_index()].SM[sm].SMflags) & ~SM_CONTROL_WATCHDOG_TRIGGER);
            }
        }

        const uint16_t configadr = ec_slave[get_index()].configadr;
        uint16_t wd_time = htoes(0);
        if (ec_FPWR(configadr, ESC_REG_SM_WATCHDOG_TIME, sizeof(wd_time), &wd_time, EC_TIMEOUTRET3) <= 0) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to write sm watchdog time (0x%04x)",
                         get_index(),
                         get_name().c_str(),
                         ESC_REG_SM_WATCHDOG_TIME);
            return 0;
        }

        // read back to make sure the watchdog is disabled
        uint16_t wd_time_read = 0xFFFF;
        if (ec_FPRD(configadr, ESC_REG_SM_WATCHDOG_TIME, sizeof(wd_time_read), &wd_time_read, EC_TIMEOUTRET3) <= 0
            || etohs(wd_time_read) != 0) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to disable sm watchdog, read back=0x%04x",
                         get_index(),
                         get_name().c_str(),
                         etohs(wd_time_read));
            return 0;
        }

        if (!ctx.is_reconfigure) {
            RCLCPP_INFO(*get_cfg_logger(),
                        "Slave id=%d (%s) sm watchdog disabled",
                        get_index(),
                        get_name().c_str());
        }

        // the slave runs in SM2 synchron mode by default, the application is triggered by every SM2 write,
        // but the master sends frames as fast as possible, far below the minimum cycle time of the slave (500us).
        // switch to free run so the slave runs on its own timer, the sync type is only writable in PRE-OP.
        // written on every reconfiguration as well, since the setting is lost after a power cycle of the slave
        if (!set_free_run(SM_OUTPUT_PARAMETER_INDEX) || !set_free_run(SM_INPUT_PARAMETER_INDEX)) {
            return 0;
        }
        if (!ctx.is_reconfigure) {
            RCLCPP_INFO(*get_cfg_logger(),
                        "Slave id=%d (%s) switched to free run",
                        get_index(),
                        get_name().c_str());
        }
        return 1;
    }

    bool SBT908E::set_free_run(const uint16_t index) const {
        if (!sdo_write<uint16_t>(index, SYNC_TYPE_SUB_INDEX, SYNC_TYPE_FREE_RUN)) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to write sync type (0x%04x:%02x)",
                         get_index(),
                         get_name().c_str(),
                         index,
                         SYNC_TYPE_SUB_INDEX);
            return false;
        }

        // read back to make sure the slave accepted it
        const std::optional<uint16_t> sync_type = sdo_read<uint16_t>(index, SYNC_TYPE_SUB_INDEX);
        if (!sync_type || *sync_type != SYNC_TYPE_FREE_RUN) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to switch to free run (0x%04x:%02x), read back=%s",
                         get_index(),
                         get_name().c_str(),
                         index,
                         SYNC_TYPE_SUB_INDEX,
                         sync_type ? fmt::format("0x{:04x}", *sync_type).c_str() : "failed");
            return false;
        }
        return true;
    }

    bool SBT908E::on_setup(SetupContext &ctx) {
        if (input_size() != INPUT_SIZE || output_size() != OUTPUT_SIZE) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) unexpected pdo size, input=%u (expected %u), output=%u (expected %u)",
                         get_index(),
                         get_name().c_str(),
                         input_size(),
                         INPUT_SIZE,
                         output_size(),
                         OUTPUT_SIZE);
            return false;
        }

        const auto channel_count = get_field_as<uint8_t>(ctx.config, get_config_prefix() + "channel_count");
        if (channel_count < 1 || channel_count > MAX_CHANNEL_COUNT) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) invalid config %schannel_count=%u, expected 1-%d",
                         get_index(),
                         get_name().c_str(),
                         get_config_prefix().c_str(),
                         channel_count,
                         MAX_CHANNEL_COUNT);
            return false;
        }
        channel_count_ = channel_count;

        // precision parameters are the same for all channels, written on every startup
        PrecisionConfig precision{};
        if (!load_precision_config(ctx.config, precision) || !apply_precision_config(precision)) {
            return false;
        }

        // the unit, the division and the calibration are set by the services, the force and peak are integers with
        // the decimal places of the division of the slave
        for (int ch = 0; ch < channel_count_; ch++) {
            if (!load_division(ch)) {
                return false;
            }
        }

        // the general data (0x6001) of the pdo carries one data type for all channels, use it for the peak value
        if (!write_param(COMM_PARA_INDEX, PDO_DATA_TYPE_SUB_INDEX, static_cast<uint8_t>(GENERAL_DATA_TYPE_PEAK),
                         "PDO_DataType")) {
            return false;
        }

        const auto pub_topic = get_field_as<std::string>(ctx.config, get_config_prefix() + "pub_topic");
        // the services are always under the publisher topic
        std::string srv_prefix = pub_topic;
        while (!srv_prefix.empty() && srv_prefix.back() == '/') {
            srv_prefix.pop_back();
        }

        // allocate once, the message is filled in the realtime thread
        custom_msgs_readsbt908e_shared_msg.channels.resize(channel_count_);
        publisher_ = ctx.node.create_publisher<custom_msgs::msg::ReadSBT908E>(
            pub_topic,
            rclcpp::SensorDataQoS()
        );

        create_command_service(ctx.node, srv_prefix + "/zero", COMMAND_ZERO);
        create_command_service(ctx.node, srv_prefix + "/clear_peak", COMMAND_CLEAR_PEAK);
        create_sdo_command_services(ctx.node, srv_prefix);
        sdo_worker_ = &ctx.sdo_worker;
        response_timer_ = ctx.node.create_wall_timer(std::chrono::milliseconds(5), [this] {
            send_command_responses();
        });

        RCLCPP_INFO(*get_cfg_logger(),
                    "Slave id=%d (%s) publishing on %s, services under %s/",
                    get_index(),
                    get_name().c_str(),
                    pub_topic.c_str(),
                    srv_prefix.c_str());
        return true;
    }

    bool SBT908E::load_precision_config(const ConfigurationParser &config, PrecisionConfig &out) const {
        const auto out_of_range = [this](const char *key, const unsigned value, const char *expected) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) invalid config %s%s=%u, expected %s",
                         get_index(),
                         get_name().c_str(),
                         get_config_prefix().c_str(),
                         key,
                         value,
                         expected);
            return false;
        };

        const std::string &prefix = get_config_prefix();

        // the values are written as they are, only checked against the ranges of the manual
        // key, destination, min, max, expected range for the log
        const std::array<std::tuple<const char *, uint8_t *, uint8_t, uint8_t, const char *>, 7> u8_fields = {
            {
                {"ad_speed", &out.ad_speed, 0, 3, "0-3"},
                {"ad_resolution", &out.ad_resolution, 16, 24, "16-24"},
                {"filter_type", &out.filter_type, 0, 10, "0-10"},
                {"filter1_level", &out.filter1_level, 0, 50, "0-50"},
                {"filter1_count", &out.filter1_count, 0, 9, "0-9"},
                {"filter2_level", &out.filter2_level, 0, 9, "0-9"},
                {"filter_stable", &out.filter_stable, 0, 1, "0/1"},
            }
        };
        for (const auto &[key, dst, min, max, expected]: u8_fields) {
            const auto value = get_field_as<uint8_t>(config, prefix + key);
            if (value < min || value > max) {
                return out_of_range(key, value, expected);
            }
            *dst = value;
        }

        return true;
    }

    bool SBT908E::apply_precision_config(const PrecisionConfig &config) {
        for (int ch = 0; ch < channel_count_; ch++) {
            const auto base = static_cast<uint16_t>(BASE_PARA_INDEX + ch);
            // AD_Speed, AD_Resolution, FilterType, Filter1Level, Filter1Count, Filter2Level, FilterStable
            const std::array<std::tuple<uint8_t, uint8_t, const char *>, 7> params = {
                {
                    {AD_SPEED_SUB_INDEX, config.ad_speed, "AD_Speed"},
                    {AD_RESOLUTION_SUB_INDEX, config.ad_resolution, "AD_Resolution"},
                    {FILTER_TYPE_SUB_INDEX, config.filter_type, "FilterType"},
                    {FILTER1_LEVEL_SUB_INDEX, config.filter1_level, "Filter1Level"},
                    {FILTER1_COUNT_SUB_INDEX, config.filter1_count, "Filter1Count"},
                    {FILTER2_LEVEL_SUB_INDEX, config.filter2_level, "Filter2Level"},
                    {FILTER_STABLE_SUB_INDEX, config.filter_stable, "FilterStable"},
                }
            };
            for (const auto &[sub_index, value, name]: params) {
                if (!write_param(base, sub_index, value, fmt::format("CH{} {}", ch + 1, name))) {
                    return false;
                }
            }

            RCLCPP_INFO(*get_cfg_logger(),
                        "Slave id=%d (%s) CH%d ad_speed=%u (%u per second), ad_resolution=%u, filter_type=%u, "
                        "filter1_level=%u, filter1_count=%u, filter2_level=%u, filter_stable=%u",
                        get_index(),
                        get_name().c_str(),
                        ch + 1,
                        config.ad_speed,
                        AD_SPEEDS[config.ad_speed],
                        config.ad_resolution,
                        config.filter_type,
                        config.filter1_level,
                        config.filter1_count,
                        config.filter2_level,
                        config.filter_stable);

            // read all of them again after everything is written, each value is only read back right after it is
            // written, a later write could make the slave change it again
            for (const auto &[sub_index, expected, name]: params) {
                const std::optional<uint8_t> value = sdo_read<uint8_t>(base, sub_index);
                if (!value || *value != expected) {
                    RCLCPP_ERROR(*get_cfg_logger(),
                                 "Slave id=%d (%s) CH%d %s (0x%04x:%02x) changed after writing all parameters, "
                                 "expected %u, read %s",
                                 get_index(),
                                 get_name().c_str(),
                                 ch + 1,
                                 name,
                                 base,
                                 sub_index,
                                 expected,
                                 value ? std::to_string(*value).c_str() : "failed");
                    return false;
                }
            }
        }
        return true;
    }

    bool SBT908E::load_division(const int ch, std::string *error) {
        const auto cali = static_cast<uint16_t>(CALI_PARA_INDEX + ch);
        const std::optional<uint8_t> div_index = sdo_read<uint8_t>(cali, DIV_SUB_INDEX);
        const std::optional<uint8_t> unit = sdo_read<uint8_t>(cali, CALI_UNIT_SUB_INDEX);
        if (!div_index || *div_index >= DIVISIONS.size() || !unit) {
            const std::string message = fmt::format("CH{} failed to read the division (0x{:04x}:{:02x}), read {}",
                                                    ch + 1, cali, DIV_SUB_INDEX,
                                                    div_index ? std::to_string(*div_index) : "failed");
            RCLCPP_ERROR(*get_cfg_logger(), "Slave id=%d (%s) %s", get_index(), get_name().c_str(), message.c_str());
            if (error != nullptr) {
                *error = message;
            }
            return false;
        }

        const int decimals = DIVISION_DECIMALS[*div_index];
        scale_[ch] = std::pow(10.0, -decimals);
        RCLCPP_INFO(*get_cfg_logger(),
                    "Slave id=%d (%s) CH%d unit=%s, div=%g (decimals=%d)",
                    get_index(),
                    get_name().c_str(),
                    ch + 1,
                    *unit < UNITS.size() ? UNITS[*unit] : "unknown",
                    DIVISIONS[*div_index],
                    decimals);
        return true;
    }

    bool SBT908E::run_sdo_command(const uint8_t channel, const uint32_t code, const int32_t para1,
                                  const int32_t para2, const int32_t para3, const std::string &name,
                                  std::string *result_message) const {
        const auto write_command = [this, channel](const uint32_t command_code, const int32_t p1, const int32_t p2,
                                                   const int32_t p3) {
            // the command code is written last, the slave executes the command when it is written
            return sdo_write<int32_t>(SDO_COMMAND_INDEX, SDO_COMMAND_CHANNEL_SUB_INDEX, channel)
                   && sdo_write<int32_t>(SDO_COMMAND_INDEX, SDO_COMMAND_PARA1_SUB_INDEX, p1)
                   && sdo_write<int32_t>(SDO_COMMAND_INDEX, SDO_COMMAND_PARA2_SUB_INDEX, p2)
                   && sdo_write<int32_t>(SDO_COMMAND_INDEX, SDO_COMMAND_PARA3_SUB_INDEX, p3)
                   && sdo_write<uint32_t>(SDO_COMMAND_INDEX, SDO_COMMAND_CODE_SUB_INDEX, command_code);
        };

        const int first_ch = channel == ALL_CHANNELS ? 0 : channel - 1;
        const int last_ch = channel == ALL_CHANNELS ? channel_count_ - 1 : channel - 1;
        std::array<uint8_t, MAX_CHANNEL_COUNT> results{};
        const auto read_results = [&] {
            // 4 bits per channel, 8 channels per Return
            std::optional<uint32_t> value;
            int value_sub_index = -1;
            for (int ch = first_ch; ch <= last_ch; ch++) {
                const int sub_index = SDO_COMMAND_RETURN1_SUB_INDEX + ch / 8;
                if (sub_index != value_sub_index) {
                    value = sdo_read<uint32_t>(SDO_COMMAND_INDEX, static_cast<uint8_t>(sub_index));
                    value_sub_index = sub_index;
                }
                results[ch] = value
                                  ? static_cast<uint8_t>(*value >> (ch % 8 * 4) & 0x0F)
                                  : COMMAND_RESULT_READ_FAILED;
            }
        };
        // e.g. "CH1=ok, CH2=error"
        const auto describe_results = [&] {
            std::string description;
            for (int ch = first_ch; ch <= last_ch; ch++) {
                if (!description.empty()) {
                    description += ", ";
                }
                description += fmt::format("CH{}={}", ch + 1, command_result_text(results[ch]));
            }
            return description;
        };
        // poll the results until all of them are cleared / reported
        const auto wait_results = [&](const bool cleared) {
            const auto deadline = std::chrono::steady_clock::now()
                                  + std::chrono::milliseconds(SDO_COMMAND_TIMEOUT_MS);
            while (true) {
                read_results();
                bool done = true;
                for (int ch = first_ch; ch <= last_ch; ch++) {
                    done &= results[ch] != COMMAND_RESULT_READ_FAILED
                            && (cleared ? results[ch] == COMMAND_RESULT_NONE : results[ch] != COMMAND_RESULT_NONE);
                }
                if (done) {
                    return true;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(SDO_COMMAND_POLL_INTERVAL_MS));
            }
        };

        const std::string target = channel == ALL_CHANNELS ? std::string("all channels") : fmt::format("CH{}", channel);
        std::string message;
        bool success = false;
        // clear the execution flags first (manual 9.4), otherwise the result of the last command would be read
        if (!write_command(COMMAND_CLEAR_FLAGS, 0, 0, 0)) {
            message = "failed to write the sdo command";
        } else if (!wait_results(true)) {
            message = "timeout clearing the execution flags, " + describe_results();
        } else if (!write_command(code, para1, para2, para3)) {
            message = "failed to write the sdo command";
        } else if (!wait_results(false)) {
            message = "timeout waiting for the execution result, " + describe_results();
        } else {
            success = true;
            for (int ch = first_ch; ch <= last_ch; ch++) {
                success &= results[ch] == COMMAND_RESULT_OK;
            }
            message = describe_results();
        }

        if (success) {
            RCLCPP_INFO(*get_cfg_logger(),
                        "Slave id=%d (%s) %s %s (command %u, para %d/%d/%d) executed, %s",
                        get_index(),
                        get_name().c_str(),
                        target.c_str(),
                        name.c_str(),
                        code,
                        para1,
                        para2,
                        para3,
                        message.c_str());
        } else {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) %s %s (command %u, para %d/%d/%d) failed, %s",
                         get_index(),
                         get_name().c_str(),
                         target.c_str(),
                         name.c_str(),
                         code,
                         para1,
                         para2,
                         para3,
                         message.c_str());
        }
        if (result_message != nullptr) {
            *result_message = std::move(message);
        }
        return success;
    }

    bool SBT908E::write_param(const uint16_t index, const uint8_t sub_index, const uint8_t value,
                              const std::string &name) const {
        const std::optional<uint8_t> current = sdo_read<uint8_t>(index, sub_index);
        if (!current) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to read %s (0x%04x:%02x)",
                         get_index(),
                         get_name().c_str(),
                         name.c_str(),
                         index,
                         sub_index);
            return false;
        }
        // the parameters are saved into the flash of the slave, don't write if nothing changed
        if (*current == value) {
            return true;
        }

        if (!sdo_write<uint8_t>(index, sub_index, value)) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to write %s (0x%04x:%02x)",
                         get_index(),
                         get_name().c_str(),
                         name.c_str(),
                         index,
                         sub_index);
            return false;
        }

        const std::optional<uint8_t> read_back = sdo_read<uint8_t>(index, sub_index);
        if (!read_back || *read_back != value) {
            RCLCPP_ERROR(*get_cfg_logger(),
                         "Slave id=%d (%s) failed to set %s (0x%04x:%02x) to %u, read back=%s",
                         get_index(),
                         get_name().c_str(),
                         name.c_str(),
                         index,
                         sub_index,
                         value,
                         read_back ? std::to_string(*read_back).c_str() : "failed");
            return false;
        }

        RCLCPP_INFO(*get_cfg_logger(),
                    "Slave id=%d (%s) %s changed %u -> %u",
                    get_index(),
                    get_name().c_str(),
                    name.c_str(),
                    *current,
                    value);
        return true;
    }

    void SBT908E::create_command_service(rclcpp::Node &node, const std::string &name, const uint32_t code) {
        // the response is deferred until the command was executed through the pdo, see send_command_responses
        create_deferred_service<CommandSrv>(
            node,
            name,
            [this, code](Responder respond, const CommandSrv::Request &request) {
                std::lock_guard lock(mtx_);
                if (reject_request(respond, request.channel)) {
                    return;
                }
                pending_commands_.push_back({
                    .respond = std::move(respond),
                    .channel = request.channel,
                    .code = code
                });
            });
    }

    void SBT908E::create_sdo_command_services(rclcpp::Node &node, const std::string &srv_prefix) {
        // sdo only commands (manual 9.6), executed by the global sdo worker, see submit_sdo_request.
        // the parameters are written as they are, only checked against the ranges of the manual
        create_deferred_service<SetCalibrationParametersSrv>(
            node,
            srv_prefix + "/set_calibration_parameters",
            [this](Responder respond, const SetCalibrationParametersSrv::Request &request) {
                std::lock_guard lock(mtx_);
                if (reject_request(respond, request.channel)) {
                    return;
                }
                if (request.unit < 0 || request.unit >= static_cast<int32_t>(UNITS.size())) {
                    fail_request(respond, fmt::format("invalid unit {}, expected 0-4", request.unit));
                    return;
                }
                if (request.div < 0 || request.div >= static_cast<int32_t>(DIVISIONS.size())) {
                    fail_request(respond, fmt::format("invalid div {}, expected 0-17", request.div));
                    return;
                }
                if (request.full_range < 0 || request.full_range > MAX_FULL_RANGE) {
                    fail_request(respond, fmt::format("invalid full_range {}, expected 0-{}", request.full_range,
                                                      MAX_FULL_RANGE));
                    return;
                }
                submit_sdo_request({
                    .respond = std::move(respond),
                    .channel = request.channel,
                    .code = COMMAND_SET_CALIBRATION,
                    .para1 = request.unit,
                    .para2 = request.div,
                    .para3 = request.full_range,
                    .name = "set calibration parameters"
                });
            });

        create_deferred_service<CommandSrv>(
            node,
            srv_prefix + "/zero_calibration",
            [this](Responder respond, const CommandSrv::Request &request) {
                std::lock_guard lock(mtx_);
                if (reject_request(respond, request.channel)) {
                    return;
                }
                submit_sdo_request({
                    .respond = std::move(respond),
                    .channel = request.channel,
                    .code = COMMAND_ZERO_CALIBRATION,
                    .para1 = 0,
                    .para2 = 0,
                    .para3 = 0,
                    .name = "zero calibration"
                });
            });

        create_deferred_service<GainCalibrationSrv>(
            node,
            srv_prefix + "/gain_calibration",
            [this](Responder respond, const GainCalibrationSrv::Request &request) {
                std::lock_guard lock(mtx_);
                if (reject_request(respond, request.channel)) {
                    return;
                }
                if (request.weight < -MAX_CALIBRATION_WEIGHT || request.weight > MAX_CALIBRATION_WEIGHT) {
                    fail_request(respond, fmt::format("invalid weight {}, expected -{} to {}", request.weight,
                                                      MAX_CALIBRATION_WEIGHT, MAX_CALIBRATION_WEIGHT));
                    return;
                }
                submit_sdo_request({
                    .respond = std::move(respond),
                    .channel = request.channel,
                    .code = COMMAND_GAIN_CALIBRATION,
                    .para1 = request.weight,
                    .para2 = 0,
                    .para3 = 0,
                    .name = "gain calibration"
                });
            });

        create_deferred_service<SensitivityCalibrationSrv>(
            node,
            srv_prefix + "/sensitivity_calibration",
            [this](Responder respond, const SensitivityCalibrationSrv::Request &request) {
                std::lock_guard lock(mtx_);
                if (reject_request(respond, request.channel)) {
                    return;
                }
                if (request.sensitivity < MIN_SENSITIVITY || request.sensitivity > MAX_SENSITIVITY) {
                    fail_request(respond, fmt::format("invalid sensitivity {}, expected {}-{}", request.sensitivity,
                                                      MIN_SENSITIVITY, MAX_SENSITIVITY));
                    return;
                }
                submit_sdo_request({
                    .respond = std::move(respond),
                    .channel = request.channel,
                    .code = COMMAND_SENSITIVITY_CALIBRATION,
                    .para1 = request.sensitivity,
                    .para2 = request.sensor_range,
                    .para3 = 0,
                    .name = "sensitivity calibration"
                });
            });
    }

    void SBT908E::fail_request(Responder &respond, std::string message) {
        finished_commands_.push_back({
            .respond = std::move(respond),
            .success = false,
            .message = std::move(message)
        });
    }

    bool SBT908E::reject_request(Responder &respond, const uint8_t channel) {
        if (channel != ALL_CHANNELS && (channel < 1 || channel > channel_count_)) {
            fail_request(respond, fmt::format("invalid channel {}, expected 1-{} or {}", channel, channel_count_,
                                              ALL_CHANNELS));
            return true;
        }
        if (!ready_) {
            fail_request(respond, "slave is not operational");
            return true;
        }
        return false;
    }

    void SBT908E::submit_sdo_request(SdoRequest request) {
        // the responder is copied into the job, so the request can still be answered if it isn't accepted
        const bool submitted = sdo_worker_->submit([this, request] {
            std::string message;
            bool success = run_sdo_command(request.channel, request.code, request.para1, request.para2,
                                           request.para3, request.name, &message);
            if (success && request.code == COMMAND_SET_CALIBRATION) {
                success = after_set_calibration(request.channel, static_cast<uint8_t>(request.para1), message);
            }
            std::lock_guard lock(mtx_);
            finished_commands_.push_back({
                .respond = request.respond,
                .success = success,
                .message = std::move(message)
            });
        });
        if (!submitted) {
            fail_request(request.respond, "sdo worker is not running");
        }
    }

    bool SBT908E::after_set_calibration(const uint8_t channel, const uint8_t unit, std::string &message) {
        const int first_ch = channel == ALL_CHANNELS ? 0 : channel - 1;
        const int last_ch = channel == ALL_CHANNELS ? channel_count_ - 1 : channel - 1;
        bool success = true;
        for (int ch = first_ch; ch <= last_ch; ch++) {
            std::string error;
            if (!write_param(static_cast<uint16_t>(BASE_PARA_INDEX + ch), BASE_UNIT_SUB_INDEX, unit,
                             fmt::format("CH{} Unit", ch + 1))) {
                message += fmt::format(", CH{} failed to write the unit of BasePara", ch + 1);
                success = false;
            }
            if (!load_division(ch, &error)) {
                message += ", " + error;
                success = false;
            }
        }
        return success;
    }

    void SBT908E::process_command(const int64_t now_ns) {
        if (command_phase_ == CommandPhase::IDLE) {
            if (pending_commands_.empty()) {
                return;
            }
            // clear the execution flags first (manual 9.4), otherwise the result of the last command would be read,
            // the code also differs from the last command, so the same command can be executed again
            write_command(pending_commands_.front().channel, COMMAND_CLEAR_FLAGS);
            command_phase_ = CommandPhase::CLEAR_FLAGS;
            command_phase_start_ns_ = now_ns;
            return;
        }

        const PendingCommand &command = pending_commands_.front();
        const int first_ch = command.channel == ALL_CHANNELS ? 0 : command.channel - 1;
        const int last_ch = command.channel == ALL_CHANNELS ? channel_count_ - 1 : command.channel - 1;
        const bool timeout = now_ns - command_phase_start_ns_ >= COMMAND_TIMEOUT_NS;

        if (command_phase_ == CommandPhase::CLEAR_FLAGS) {
            bool cleared = true;
            for (int ch = first_ch; ch <= last_ch; ch++) {
                cleared &= read_command_result(ch) == COMMAND_RESULT_NONE;
            }
            if (cleared) {
                write_command(command.channel, command.code);
                command_phase_ = CommandPhase::EXECUTE;
                command_phase_start_ns_ = now_ns;
            } else if (timeout) {
                finish_command(false, "timeout clearing the execution flags, "
                                      + describe_command_results(first_ch, last_ch));
            }
            return;
        }

        // EXECUTE, wait until every channel reported a result
        bool executed = true;
        bool ok = true;
        for (int ch = first_ch; ch <= last_ch; ch++) {
            const uint8_t result = read_command_result(ch);
            executed &= result != COMMAND_RESULT_NONE;
            ok &= result == COMMAND_RESULT_OK;
        }
        if (executed) {
            finish_command(ok, describe_command_results(first_ch, last_ch));
        } else if (timeout) {
            finish_command(false, "timeout waiting for the execution result, "
                                  + describe_command_results(first_ch, last_ch));
        }
    }

    void SBT908E::write_command(const uint8_t channel, const uint32_t code) {
        uint8_t *buf = tx_buf().data();
        int offset = COMMAND_CHANNEL_OFFSET;
        write_int32(channel, buf, &offset);
        offset = COMMAND_CODE_OFFSET;
        write_uint32(code, buf, &offset);
    }

    uint8_t SBT908E::read_command_result(const int ch) {
        // 8 channels per Return
        int offset = COMMAND_RETURN_OFFSET + ch / 8 * 4;
        const uint32_t value = read_uint32(rx_buf().data(), &offset);
        return static_cast<uint8_t>(value >> (ch % 8 * 4) & 0x0F);
    }

    std::string SBT908E::describe_command_results(const int first_ch, const int last_ch) {
        std::string description;
        for (int ch = first_ch; ch <= last_ch; ch++) {
            if (!description.empty()) {
                description += ", ";
            }
            description += fmt::format("CH{}={}", ch + 1, command_result_text(read_command_result(ch)));
        }
        return description;
    }

    void SBT908E::finish_command(const bool success, std::string message) {
        // the result is checked, reset the code to 0
        int offset = COMMAND_CODE_OFFSET;
        write_uint32(COMMAND_NONE, tx_buf().data(), &offset);
        command_phase_ = CommandPhase::IDLE;

        PendingCommand command = std::move(pending_commands_.front());
        pending_commands_.pop_front();
        finished_commands_.push_back({
            .respond = std::move(command.respond),
            .success = success,
            .message = std::move(message)
        });
    }

    void SBT908E::fail_all_commands(const std::string &reason) {
        if (command_phase_ != CommandPhase::IDLE) {
            int offset = COMMAND_CODE_OFFSET;
            write_uint32(COMMAND_NONE, tx_buf().data(), &offset);
            command_phase_ = CommandPhase::IDLE;
        }
        while (!pending_commands_.empty()) {
            PendingCommand command = std::move(pending_commands_.front());
            pending_commands_.pop_front();
            finished_commands_.push_back({
                .respond = std::move(command.respond),
                .success = false,
                .message = reason
            });
        }
    }

    void SBT908E::send_command_responses() {
        std::deque<FinishedCommand> finished;
        {
            std::lock_guard lock(mtx_);
            finished.swap(finished_commands_);
        }

        for (auto &command: finished) {
            try {
                command.respond(command.success, command.message);
            } catch (const std::exception &e) {
                RCLCPP_WARN(*get_data_logger(),
                            "Slave id=%d (%s) failed to send service response: %s",
                            get_index(),
                            get_name().c_str(),
                            e.what());
            }
        }
    }

    void SBT908E::on_cycle(const CycleContext &ctx) {
        if (!ready_) {
            if (command_phase_ != CommandPhase::IDLE || !pending_commands_.empty()) {
                fail_all_commands("slave is not operational");
            }
            return;
        }

        const int64_t now_ns = ctx.now.nanoseconds();
        process_command(now_ns);

        if (now_ns - last_publish_ns_ < PUBLISH_INTERVAL_NS) {
            return;
        }
        last_publish_ns_ = now_ns;

        custom_msgs_readsbt908e_shared_msg.header.stamp = ctx.now;

        const uint8_t *buf = rx_buf().data();
        int type_offset = GENERAL_DATA_TYPE_OFFSET;
        // the general data type can be changed on the panel (F7-02), only trust it if it is the peak value
        const bool peak_valid = read_uint16(buf, &type_offset) == GENERAL_DATA_TYPE_PEAK;

        int force_offset = FORCE_GROSS_OFFSET;
        int peak_offset = GENERAL_DATA_VALUE_OFFSET;
        for (int ch = 0; ch < channel_count_; ch++) {
            auto &channel = custom_msgs_readsbt908e_shared_msg.channels[ch];
            const double scale = scale_[ch].load(std::memory_order_relaxed);
            channel.force = read_int32(buf, &force_offset) * scale;
            const int32_t peak = read_int32(buf, &peak_offset);
            channel.peak = peak_valid ? peak * scale : std::numeric_limits<double>::quiet_NaN();
        }

        publisher_->publish(custom_msgs_readsbt908e_shared_msg);
    }

    void SBT908E::cleanup() {
        // the executor is already stopped, pending requests can't be answered anymore
        pending_commands_.clear();
        finished_commands_.clear();
        response_timer_.reset();
        services_.clear();
        publisher_.reset();
    }
}
