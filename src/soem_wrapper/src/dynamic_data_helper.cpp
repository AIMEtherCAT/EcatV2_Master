// ReSharper disable CppCStyleCast
#include "soem_wrapper/dynamic_data_helper.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::utils::dynamic_data {
    using namespace ecat::logging;

    DynamicStruct dynamic_data{};

    DynamicStruct *get_dynamic_data() {
        return &dynamic_data;
    }

    DynamicStruct::DynamicStruct(std::initializer_list<std::pair<std::string, Variant> > init) {
        for (const auto &[key, value]: init) {
            fields_[key] = value;
        }
        memset(temp_buf_, 0, 1024);
    }

    void DynamicStruct::parse_map(const std::string &path, const YAML::Node &node) {
        for (const auto &kv: node) {
            auto key = kv.first.as<std::string>();
            if (const YAML::Node &val = kv.second; val.IsScalar()) {
                const std::string insert_key = fmt::format("{}_{}", path, key);
                if (const std::string &tag = val.Tag(); tag == "!uint8_t") {
                    set(insert_key, val.as<uint8_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type uint8_t key %s value %u", insert_key.c_str(),
                                 val.as<uint8_t>());
                } else if (tag == "!int8_t") {
                    set(insert_key, val.as<int8_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type int8_t key %s value %d", insert_key.c_str(),
                                 val.as<int8_t>());
                } else if (tag == "!uint16_t") {
                    set(insert_key, val.as<uint16_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type uint16_t key %s value %u", insert_key.c_str(),
                                 val.as<uint16_t>());
                } else if (tag == "!int16_t") {
                    set(insert_key, val.as<int16_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type int16_t key %s value %d", insert_key.c_str(),
                                 val.as<int16_t>());
                } else if (tag == "!uint32_t") {
                    set(insert_key, val.as<uint32_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type uint32_t key %s value %u", insert_key.c_str(),
                                 val.as<uint32_t>());
                } else if (tag == "!int32_t") {
                    set(insert_key, val.as<int32_t>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type int32_t key %s value %d", insert_key.c_str(),
                                 val.as<int32_t>());
                } else if (tag == "!float") {
                    set(insert_key, val.as<float>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type float key %s value %f", insert_key.c_str(),
                                 val.as<float>());
                } else if (tag == "!std::string") {
                    set(insert_key, val.as<std::string>());
                    RCLCPP_DEBUG(*get_data_logger(), "YAML insert type std::string key %s value %s", insert_key.c_str(),
                                 val.as<std::string>().c_str());
                } else {
                    throw std::runtime_error("Unsupported tag: " + tag);
                }
            } else if (val.IsSequence()) {
                for (auto &&slave: val) {
                    if (slave.IsMap()) {
                        for (auto it = slave.begin(); it != slave.end(); ++it) {
                            parse_map(fmt::format("{}_{}", path, it->first.as<std::string>()), it->second);
                        }
                    }
                }
            }
        }
    }

    void DynamicStruct::load_initial_value_from_config(const std::string &filepath) {
        if (const YAML::Node root = YAML::LoadFile(filepath); root["slaves"] && root["slaves"].IsSequence()) {
            for (YAML::Node slaves = root["slaves"]; auto &&slave: slaves) {
                if (slave.IsMap()) {
                    for (auto it = slave.begin(); it != slave.end(); ++it) {
                        parse_map(it->first.as<std::string>(), it->second);
                    }
                }
            }
        }
    }

    void DynamicStruct::set(const std::string &key, Variant value) {
        fields_[key] = std::move(value);
    }

    Variant DynamicStruct::get(const std::string &key) const {
        if (const auto it = fields_.find(key);
            it != fields_.end())
            return it->second;
        throw std::runtime_error("get Key not found: " + key);
    }

    bool DynamicStruct::has(const std::string &key) const {
        return fields_.contains(key);
    }

    void DynamicStruct::remove(const std::string &key) {
        if (const auto it = fields_.find(key);
            it != fields_.end()) {
            fields_.erase(it);
        } else {
            throw std::runtime_error("del Key not found: " + key);
        }
    }

    BuildBufResultT DynamicStruct::build_buf(const std::string &prefix, const std::vector<std::string> &names) {
        offset_ = 0;
        memset(temp_buf_, 0, sizeof(temp_buf_));

        for (const std::string &name: names) {
            std::visit(
                [&, this]<typename T>(T &&arg) {
                    if constexpr (std::is_same_v<T, uint8_t>) {
                        write_uint8(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, int8_t>) {
                        write_int8(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, uint16_t>) {
                        write_uint16(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, int16_t>) {
                        write_int16(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, uint32_t>) {
                        write_uint32(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, int32_t>) {
                        write_int32(arg, temp_buf_, &offset_);
                    } else if constexpr (std::is_same_v<T, float>) {
                        write_float(arg, temp_buf_, &offset_);
                    }
                },
                get(fmt::format("{}{}", prefix, name))
            );
        }

        return BuildBufResultT{temp_buf_, offset_};
    }

    void DynamicStruct::reset_and_remove_publishers() {
        std::vector<std::string> keys_to_remove;

        for (auto &[key, value]: fields_) {
            // all pub/sub inst ends with _inst
            if (key.size() >= 5 && key.compare(key.size() - 5, 5, "_inst") == 0) {
                std::visit(
                    [&]<typename T>(T &inst) {
                        if constexpr (is_publisher_or_subscriber<T>::value) {
                            if (inst) {
                                inst.reset();
                                keys_to_remove.push_back(key);
                            }
                        }
                    },
                    value
                );
            }
        }

        for (const auto &key: keys_to_remove) {
            fields_.erase(key);
        }
    }
}
