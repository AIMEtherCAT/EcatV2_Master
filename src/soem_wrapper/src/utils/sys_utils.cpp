//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/utils/sys_utils.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <regex>
#include <fstream>
#include <sstream>

namespace aim::utils::sys {
    using namespace ecat::logging;

    std::string exec_cmd(const std::string &cmd) {
        std::array<char, 128> buffer{};
        std::string result;
        FILE *pipe = popen(cmd.c_str(), "r");
        if (!pipe) return "ERROR";
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            result.append(buffer.data(), std::strlen(buffer.data()));
        }
        pclose(pipe);
        return result;
    }

    void move_threads(const int cpu_id, const std::string &cpu_list, const std::string &nic_name) {
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
                    RCLCPP_DEBUG(*get_sys_logger(), "Keep %d %s at cpu %d", pid, comm.c_str(), cpu_id);
                } else {
                    RCLCPP_DEBUG(*get_sys_logger(),
                                 "Move %d %s to cpu %s", pid, comm.c_str(), cpu_list.c_str());
                    std::string taskset_cmd = "sudo taskset -cp " + cpu_list + " " + std::to_string(pid) +
                                              " > /dev/null 2>&1";
                    system(taskset_cmd.c_str());
                }
            }
        }
    }

    void move_irq(const int cpu_id, const std::string &nic_name) {
        unsigned int cpu_mask = 1U << cpu_id;
        std::string line;

        std::ifstream interrupt_file("/proc/interrupts");
        while (getline(interrupt_file, line)) {
            if (line.find(nic_name) != std::string::npos) {
                std::stringstream ss(line);
                std::string irq_str;
                ss >> irq_str;
                irq_str = regex_replace(irq_str, std::regex(":"), "");
                std::string desc = line.substr(line.find(':') + 1);
                std::string affinity_path = "/proc/irq/" + irq_str + "/smp_affinity";
                if (std::ofstream affinity_file(affinity_path); affinity_file) {
                    affinity_file << std::hex << cpu_mask;
                    RCLCPP_DEBUG(*get_sys_logger(),
                                 "Bind irq %s %s to cpu %d successfully",
                                 irq_str.c_str(),
                                 desc.c_str(),
                                 cpu_id);
                } else {
                    RCLCPP_ERROR(*get_sys_logger(),
                                 "Bind irq %s %s to cpu %d failed",
                                 irq_str.c_str(),
                                 desc.c_str(),
                                 cpu_id);
                }
            }
        }
    }

    void setup_nic(const std::string &nic_name) {
        const int sock = socket(AF_INET, SOCK_DGRAM, 0);
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
                RCLCPP_WARN(*get_sys_logger(), "NIC %s %s: read current coalesce failed: %s",
                            nic_name.c_str(), desc.c_str(), strerror(errno));
                return;
            }

            modify_fn(ecoal);

            ecoal.cmd = ETHTOOL_SCOALESCE;
            ifr.ifr_data = reinterpret_cast<char *>(&ecoal);
            if (ioctl(sock, SIOCETHTOOL, &ifr) < 0) {
                RCLCPP_WARN(*get_sys_logger(), "NIC %s %s update failed: %s", nic_name.c_str(), desc.c_str(),
                            strerror(errno));
            } else {
                RCLCPP_DEBUG(*get_sys_logger(), "NIC %s %s updated", nic_name.c_str(), desc.c_str());
            }
        };

        set_coalesce_param([](ethtool_coalesce &e) { e.rx_coalesce_usecs = 0; }, "rx_coalesce_usecs");
        set_coalesce_param([](ethtool_coalesce &e) { e.tx_coalesce_usecs = 0; }, "tx_coalesce_usecs");
        set_coalesce_param([](ethtool_coalesce &e) { e.rx_max_coalesced_frames = 1; }, "rx_max_coalesced_frames");
        set_coalesce_param([](ethtool_coalesce &e) { e.tx_max_coalesced_frames = 1; }, "tx_max_coalesced_frames");

        close(sock);
        RCLCPP_INFO(*get_sys_logger(), "NIC %s low-latency params setup done", nic_name.c_str());
    }
}
