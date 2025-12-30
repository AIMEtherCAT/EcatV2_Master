//
// Created by hang on 12/26/25.
//

#ifndef BUILD_SYS_UTILS_H
#define BUILD_SYS_UTILS_H

#include <string>

namespace aim::utils::sys {
    std::string exec_cmd(const std::string &cmd);

    void move_threads(int cpu_id, const std::string &cpu_list, const std::string &nic_name);

    void move_irq(int cpu_id, const std::string &nic_name);

    void setup_nic(const std::string &nic_name);
}

#endif //BUILD_SYS_UTILS_H