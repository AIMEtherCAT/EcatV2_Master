//
// Created by hang on 10/10/26.
//

#ifndef SOEM_WRAPPER_DEVICE_DEVICE_REGISTRY_HPP
#define SOEM_WRAPPER_DEVICE_DEVICE_REGISTRY_HPP

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "soem_wrapper/device/slave_device.hpp"

namespace aim::ecat::device {
    using DeviceFactory = std::function<std::shared_ptr<SlaveDevice>(const DeviceInfo &)>;

    /**
     * maps (vendor id, product code) from the slave eeprom to the device implementation
     */
    class DeviceRegistry {
    public:
        struct Entry {
            std::string name;
            DeviceFactory factory;
        };

        void add(uint32_t vendor_id, uint32_t product_code, const std::string &name, DeviceFactory factory);

        [[nodiscard]] const Entry *find(uint32_t vendor_id, uint32_t product_code) const;

    private:
        std::map<std::pair<uint32_t, uint32_t>, Entry> entries_{};
    };
}

#endif //SOEM_WRAPPER_DEVICE_DEVICE_REGISTRY_HPP
