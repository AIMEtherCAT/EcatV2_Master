//
// Created by hang on 10/10/26.
//
#include "soem_wrapper/device/device_registry.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat::device {
    void DeviceRegistry::add(const uint32_t vendor_id, const uint32_t product_code, const std::string &name,
                             DeviceFactory factory) {
        RCLCPP_INFO(*logging::get_wrapper_logger(),
                    "Registered new device, vendor=0x%08x, product=0x%08x, name=%s",
                    vendor_id,
                    product_code,
                    name.c_str());
        entries_[{vendor_id, product_code}] = Entry{
            .name = name,
            .factory = std::move(factory)
        };
    }

    const DeviceRegistry::Entry *DeviceRegistry::find(const uint32_t vendor_id, const uint32_t product_code) const {
        if (const auto it = entries_.find({vendor_id, product_code}); it != entries_.end()) {
            return &it->second;
        }
        return nullptr;
    }
}
