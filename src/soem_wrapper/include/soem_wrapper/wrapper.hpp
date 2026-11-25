//
// Created by hang on 12/26/25.
//

#ifndef BUILD_SOEM_WRAPPER_H
#define BUILD_SOEM_WRAPPER_H

#include "soem_wrapper/device/slave_device.hpp"
#include "soem_wrapper/device/aim_universal_module.hpp"

typedef uint16_t uint16;
typedef struct ecx_context ecx_contextt;

namespace aim::ecat {
    /**
     * create one device per slave on the bus according to the device registry
     * @return false if any slave is not registered
     */
    bool init_slave_devices_vector(int slave_count);

    std::shared_ptr<device::SlaveDevice> get_slave_device(int index);

    std::vector<std::shared_ptr<device::SlaveDevice> > &get_slave_devices();

    int config_ec_slave(ecx_contextt * /*context*/, uint16 slave);
}

#endif //BUILD_SOEM_WRAPPER_H
