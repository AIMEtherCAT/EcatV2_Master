//
// Created by hang on 25-6-27.
//

#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "ethercattype.h"
#include "nicdrv.h"
#include "osal.h"
#include "ethercatmain.h"
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "fmt/core.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <dirent.h>
#include <cstdlib>
#include <bitset>

#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>

// these header files will be used by soem_backend.cpp main func
// ReSharper disable once CppUnusedIncludeDirective
#include "ethercatconfig.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "ethercatdc.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "ethercatcoe.h"

// these header files will be used by other *_defs.hpp files
// ReSharper disable once CppUnusedIncludeDirective
#include "defs/packet_defs.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "utils.hpp"

using namespace std::chrono_literals;

extern rclcpp::Logger data_logger;
extern DynamicStruct sdo_data;
extern rclcpp::Logger wrapper_logger;
extern rclcpp::Logger cfg_logger;
extern rclcpp::Logger sys_logger;

template<typename T>
T get_field_as(const uint32_t sn, const uint16_t app_idx, const std::string &suffix, const T &default_value) {
    try {
        return std::get<T>(sdo_data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
    } catch (const std::runtime_error &) {
        return default_value;
    }
}

template<typename T>
T get_field_as(const uint32_t sn, const std::string &suffix, const T &default_value) {
    try {
        return std::get<T>(sdo_data.get(fmt::format("sn{}_{}", sn, suffix)));
    } catch (const std::runtime_error &) {
        return default_value;
    }
}

template<typename T>
T get_field_as(const std::string &whole_key, const T &default_value) {
    try {
        return std::get<T>(sdo_data.get(whole_key));
    } catch (const std::runtime_error &) {
        return default_value;
    }
}

template<typename T>
T get_field_as(const uint32_t sn, const uint16_t app_idx, const std::string &suffix) {
    return std::get<T>(sdo_data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
}

template<typename T>
T get_field_as(const uint32_t sn, const std::string &suffix) {
    return std::get<T>(sdo_data.get(fmt::format("sn{}_{}", sn, suffix)));
}

template<typename T>
T get_field_as(const std::string &whole_key) {
    return std::get<T>(sdo_data.get(whole_key));
}

template<typename MsgT>
struct MsgDef {
    static constexpr auto type_enum = "Unknown";

    static void
    write(const typename MsgT::SharedPtr & /*msg*/, uint8_t * /*buf*/, int * /*offset*/, const std::string &prefix) {
        RCLCPP_ERROR(data_logger, "Unsupported MsgT in generic_callback for prefix: %s", prefix.c_str());
    }
};

struct AppDef {
    virtual ~AppDef() = default;

    virtual void init_sdo(uint8_t * /*buf*/, int * /*offset*/, uint32_t /*sn*/, uint8_t /*slave_id*/,
                          const std::string & /*prefix*/) = 0;

    virtual void init_value(uint8_t * /*buf*/, int * /*offset*/, const std::string & /*prefix*/) = 0;

    virtual void read(const uint8_t * /*buf*/, int * /*offset*/, const std::string & /*prefix*/) = 0;
};

template<typename, typename = std::void_t<> >
struct has_init_sdo : std::false_type {
};

template<typename T>
struct has_init_sdo<T, std::void_t<
            decltype(T::init_sdo(std::declval<uint8_t *>(),
                                 std::declval<int *>(),
                                 std::declval<uint32_t>(),
                                 std::declval<uint8_t>(),
                                 std::declval<std::string>()))> > : std::true_type {
};

template<typename, typename = std::void_t<> >
struct has_init_value : std::false_type {
};

template<typename T>
struct has_init_value<T, std::void_t<
            decltype(T::init_value(std::declval<uint8_t *>(),
                                   std::declval<int *>(),
                                   std::declval<std::string>()))> > : std::true_type {
};

template<typename, typename = std::void_t<> >
struct has_read : std::false_type {
};

template<typename T>
struct has_read<T, std::void_t<
            decltype(T::read(std::declval<uint8_t *>(),
                             std::declval<int *>(),
                             std::declval<std::string>()))> > : std::true_type {
};


template<typename AppT>
struct AppWrapper final : AppDef {
    void
    init_sdo(uint8_t *buf, int *offset, uint32_t sn, uint8_t slave_id, const std::string &prefix) override {
        call_init_sdo<AppT>(buf, offset, sn, slave_id, prefix);
    }

    void
    init_value(uint8_t *buf, int *offset, const std::string &prefix) override {
        call_init_value<AppT>(buf, offset, prefix);
    }

    void
    read(const uint8_t *buf, int *offset, const std::string &prefix) override {
        call_read<AppT>(buf, offset, prefix);
    }

private:
    template<typename T, std::enable_if_t<has_init_sdo<T>::value, int> = 0>
    static void
    call_init_sdo(uint8_t *buf, int *offset, uint32_t sn, uint8_t slave_id, const std::string &prefix) {
        T::init_sdo(buf, offset, sn, slave_id, prefix);
    }

    template<typename T, std::enable_if_t<!has_init_sdo<T>::value, int> = 0>
    static void
    call_init_sdo(uint8_t *, int *, uint32_t, uint8_t, const std::string &) {
        RCLCPP_WARN(wrapper_logger, "App name=%s don't have init_sdo method but got called", T::type_enum);
    }

    template<typename T, std::enable_if_t<has_init_value<T>::value, int> = 0>
    static void
    call_init_value(uint8_t *buf, int *offset, const std::string &prefix) {
        T::init_value(buf, offset, prefix);
    }

    template<typename T, std::enable_if_t<!has_init_value<T>::value, int> = 0>
    static void
    call_init_value(uint8_t *, int *, const std::string &) {
        RCLCPP_WARN(wrapper_logger, "App name=%s don't have init_value method but got called", T::type_enum);
    }

    template<typename T, std::enable_if_t<has_read<T>::value, int> = 0>
    static void
    call_read(const uint8_t *buf, int *offset, const std::string &prefix) {
        T::read(buf, offset, prefix);
    }

    template<typename T, std::enable_if_t<!has_read<T>::value, int> = 0>
    static void
    call_read(const uint8_t *, int *, const std::string &) {
        // RCLCPP_WARN(wrapper_logger, "App name=%s don't have read method but got called", T::type_enum);
    }
};


struct slave_device {
    ec_slavet *slave = nullptr;
    uint32_t sn = 0;
    uint8_t device_type = 0;

    uint8_t ready = 0;
    uint8_t waiting_for_latency_checking = 0;
    rclcpp::Time last_packet_time{};

    uint8_t master_status = 0;
    std::vector<uint8_t> master_to_slave_buf{};
    uint16_t master_to_slave_buf_len = 0;

    uint8_t slave_status = 0;
    std::vector<uint8_t> slave_to_master_buf{};
    uint16_t slave_to_master_buf_len = 0;

    std::vector<uint8_t> arg_buf{};
    uint16_t arg_buf_len = 0;
    int arg_buf_idx = 0;

    uint16_t sending_arg_buf_idx = 1;
    uint8_t arg_sent = 0;

    // ecat conf done flag
    uint8_t conf_done = 0;
    // ros2 pub/sub conf done flag
    uint8_t conf_ros_done = 0;
};

extern slave_device slave_devices[EC_MAXSLAVE];

class EthercatNode final : public rclcpp::Node {
public:
    EthercatNode() : Node("EthercatNode"), running_(true) {
        this->declare_parameter<std::string>("interface", "enp2s0");
        interface = this->get_parameter("interface").as_string();
        RCLCPP_INFO(this->get_logger(), "Using interface: %s", interface.c_str());

        this->declare_parameter<int>("rt_cpu", 6);
        rt_cpu = this->get_parameter("rt_cpu").as_int();
        RCLCPP_INFO(this->get_logger(), "Using rt-cpu: %d", rt_cpu);

        this->declare_parameter<std::string>("non_rt_cpus", "0-5,7-15");
        non_rt_cpus = this->get_parameter("non_rt_cpus").as_string();
        RCLCPP_INFO(this->get_logger(), "Using non_rt_cpus: %s", non_rt_cpus.c_str());

        this->declare_parameter<std::string>("config_file", "/home/txy/ecat_ws/src/soem_wrapper/config/config.yaml");
        config_file = this->get_parameter("config_file").as_string();
        RCLCPP_INFO(this->get_logger(), "Using config_file: %s", config_file.c_str());
        sdo_data.load_initial_value_from_config(config_file);
        RCLCPP_INFO(data_logger, "Configuration file loaded");

        register_components();
    }

    ~EthercatNode() override;

    std::string
    get_device_name(const uint32 eep_id) {
        return registered_module_names[eep_id];
    }

    uint16_t
    get_device_master_to_slave_buf_len(const uint32 eep_id) {
        return registered_module_buf_lens[eep_id].first;
    }

    uint16_t
    get_device_slave_to_master_buf_len(const uint32 eep_id) {
        return registered_module_buf_lens[eep_id].second;
    }

    template<typename MsgT>
    void
    create_and_insert_publisher(const std::string &prefix) {
        sdo_data.set(
            fmt::format("{}pub_inst", prefix),
            this->create_publisher<MsgT>(
                get_field_as<std::string>(fmt::format("{}pub_topic", prefix)),
                rclcpp::SensorDataQoS()
            )
        );
        RCLCPP_DEBUG(wrapper_logger, "Publisher created key=%spub_inst", prefix.c_str());
    }

    template<typename MsgT>
    static void
    publish_msg(const std::string &prefix, const MsgT &msg) {
        std::get<typename rclcpp::Publisher<MsgT>::SharedPtr>(sdo_data.get(
            fmt::format("{}pub_inst", prefix)
        ))->publish(msg);
    }

    template<typename MsgT>
    void
    create_and_insert_subscriber(const std::string &prefix, const uint8_t slave_id) {
        sdo_data.set(
            fmt::format("{}sub_inst", prefix),
            this->create_subscription<MsgT>(
                get_field_as<std::string>(fmt::format("{}sub_topic", prefix)),
                rclcpp::SensorDataQoS(),
                [=](const typename MsgT::SharedPtr msg) {
                    generic_callback<MsgT>(msg, prefix, slave_id);
                }
            )
        );
        RCLCPP_DEBUG(wrapper_logger, "Subscriber created key=%ssub_inst", prefix.c_str());
    }

    /**
     * ros2 msg subscriber callback func
     * write data to corresponding PDO buffer
     * @tparam MsgT ros2 msg type
     * @param msg msg ref
     * @param prefix dynamic struct key prefix
     * @param slave_id slave id
     */
    template<typename MsgT>
    void
    generic_callback(const typename MsgT::SharedPtr &msg, const std::string &prefix, const uint8_t slave_id) {
        pdo_offset_for_cb = get_field_as<uint16_t>(fmt::format("{}pdowrite_offset", prefix));
        offset_for_cb = pdo_offset_for_cb;

        MsgDef<MsgT>::write(msg, slave_devices[slave_id].master_to_slave_buf.data(), &offset_for_cb, prefix);
    }

    std::string interface{};
    int rt_cpu{};
    std::string non_rt_cpus{};
    std::string config_file{};

    // m2s, s2m
    std::unordered_map<uint32_t, std::pair<uint16_t, uint16_t> > registered_module_buf_lens{};
    std::unordered_map<uint32_t, std::string> registered_module_names{};
    std::unordered_map<uint32_t, std::unique_ptr<AppDef> > app_registry{};

    bool setup_ethercat(const char *);

    void register_components();

private:
    void
    register_module(const uint32_t eep_id, const std::string &module_name,
                    const int master_to_slave_buf_len,
                    const int slave_to_master_buf_len) {
        RCLCPP_INFO(wrapper_logger, "Registered new module, eepid=%d, name=%s, m2slen=%d, s2mlen=%d", eep_id,
                    module_name.c_str(), master_to_slave_buf_len, slave_to_master_buf_len);
        registered_module_names[eep_id] = module_name;
        registered_module_buf_lens[eep_id] =
                std::make_pair(master_to_slave_buf_len, slave_to_master_buf_len);
    }

    template<typename AppT>
    void
    register_app() {
        RCLCPP_INFO(wrapper_logger, "Registered new app, id=%d, name=%s", AppT::type_id, AppT::type_enum);
        app_registry[AppT::type_id] = std::make_unique<AppWrapper<AppT> >();
    }

    static std::string exec_cmd(const std::string &cmd) {
        std::array<char, 128> buffer;
        std::string result;
        FILE *pipe = popen(cmd.c_str(), "r");
        if (!pipe) return "ERROR";
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            result += buffer.data();
        }
        pclose(pipe);
        return result.c_str();
    }

    static void move_threads(const int cpu_id, const std::string &cpu_list, const std::string &nic_name) {
        std::string ps_output = exec_cmd("ps -eLo pid,psr,comm --no-headers");
        std::istringstream ps_stream(ps_output);
        std::string line;
        while (getline(ps_stream, line)) {
            std::istringstream linestream(line);
            int pid, psr;
            std::string comm;
            linestream >> pid >> psr >> comm;

            if (psr == cpu_id) {
                if (comm == "soem_backend" || comm.find(nic_name) != std::string::npos) {
                    RCLCPP_DEBUG(sys_logger, "Keep %d %s at cpu %d", pid, comm.c_str(), cpu_id);
                } else {
                    RCLCPP_DEBUG(sys_logger, "Move %d %s to cpu %s", pid, comm.c_str(), cpu_list.c_str());
                    std::string taskset_cmd = "sudo taskset -cp " + cpu_list + " " + std::to_string(pid) +
                                              " > /dev/null 2>&1";
                    system(taskset_cmd.c_str());
                }
            }
        }
    }

    static void move_irq(const int cpu_id, const std::string &nic_name) {
        unsigned int cpu_mask = 1U << cpu_id;
        std::string line;

        std::ifstream interrupt_file("/proc/interrupts");
        while (getline(interrupt_file, line)) {
            if (line.find(nic_name) != std::string::npos) {
                std::stringstream ss(line);
                std::string irq_str;
                ss >> irq_str;
                irq_str = regex_replace(irq_str, std::regex(":"), "");
                std::string desc = line.substr(line.find(":") + 1);
                std::string affinity_path = "/proc/irq/" + irq_str + "/smp_affinity";
                std::ofstream affinity_file(affinity_path);
                if (affinity_file) {
                    affinity_file << std::hex << cpu_mask;
                    RCLCPP_DEBUG(sys_logger, "Bind irq %s %s to cpu %d successfully", irq_str.c_str(), desc.c_str(),
                                 cpu_id);
                } else {
                    RCLCPP_ERROR(sys_logger, "Bind irq %s %s to cpu %d failed", irq_str.c_str(), desc.c_str(), cpu_id);
                }
            }
        }
    }

    static void setup_nic(const std::string &nic_name) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("socket");
            exit(EXIT_FAILURE);
        }

        ifreq ifr{};
        ethtool_coalesce ecoal{};
        std::strncpy(ifr.ifr_name, nic_name.c_str(), IFNAMSIZ - 1);

        auto set_coalesce_param = [&](auto modify_fn, const std::string &desc) {
            std::memset(&ecoal, 0, sizeof(ecoal));
            ecoal.cmd = ETHTOOL_GCOALESCE;
            ifr.ifr_data = reinterpret_cast<char *>(&ecoal);

            if (ioctl(sock, SIOCETHTOOL, &ifr) < 0) {
                RCLCPP_WARN(sys_logger, "NIC %s %s: read current coalesce failed: %s",
                            nic_name.c_str(), desc.c_str(), strerror(errno));
                return;
            }

            modify_fn(ecoal);

            ecoal.cmd = ETHTOOL_SCOALESCE;
            ifr.ifr_data = reinterpret_cast<char *>(&ecoal);
            if (ioctl(sock, SIOCETHTOOL, &ifr) < 0) {
                RCLCPP_WARN(sys_logger, "NIC %s %s update failed: %s", nic_name.c_str(), desc.c_str(), strerror(errno));
            } else {
                RCLCPP_DEBUG(sys_logger, "NIC %s %s updated", nic_name.c_str(), desc.c_str());
            }
        };

        set_coalesce_param([](ethtool_coalesce &e) { e.rx_coalesce_usecs = 0; }, "rx_coalesce_usecs");
        set_coalesce_param([](ethtool_coalesce &e) { e.tx_coalesce_usecs = 0; }, "tx_coalesce_usecs");
        set_coalesce_param([](ethtool_coalesce &e) { e.rx_max_coalesced_frames = 1; }, "rx_max_coalesced_frames");
        set_coalesce_param([](ethtool_coalesce &e) { e.tx_max_coalesced_frames = 1; }, "tx_max_coalesced_frames");

        close(sock);
        RCLCPP_INFO(sys_logger, "NIC %s low-latency params setup done", nic_name.c_str());
    }

    void datacycle_callback();

    char IOmap[4096]{};

    bool pdo_transfer_active = false;
    std::atomic<bool> running_{};
    std::thread worker_thread_{};

    uint16_t pdo_offset_for_cb = 0;
    int offset_for_cb = 0;
};

extern std::shared_ptr<EthercatNode> node;

#endif //ETHERCAT_NODE_HPP
