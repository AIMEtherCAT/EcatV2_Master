//
// Created by hang on 25-5-4.
//

#ifndef CONFIGURATION_PARSER_H
#define CONFIGURATION_PARSER_H

#include <iostream>
#include <unordered_map>
#include <utility>
#include <variant>
#include <string>
#include <vector>

#include "fmt/core.h"
#include "fmt/format.h"
#include "yaml-cpp/yaml.h"

namespace aim::utils::config {
    using Variant = std::variant<
        uint8_t,
        uint16_t,
        int8_t,
        int16_t,
        uint32_t,
        int32_t,
        float,
        std::string,
        bool>;

    struct BuildBufResultT {
        uint8_t *buf;
        int len;
    };

    class ConfigurationParser {
    public:
        ConfigurationParser(std::initializer_list<std::pair<std::string, Variant> > init);

        void load_initial_value_from_config(const std::string &filepath);

        void parse_map(const std::string &path, const YAML::Node &node);

        void set(const std::string &key, Variant value);

        Variant get(const std::string &key) const;

        bool has(const std::string &key) const;

        void remove(const std::string &key);

        /**
         * copy all values that matches the given full name into a buffer
         * this method is NOT thread-safe
         *
         * @param prefix universal given prefix
         * @param names names
         * @return buffer with corresponding values
         */
        BuildBufResultT build_buf(const std::string &prefix, const std::vector<std::string> &names);

    private:
        std::unordered_map<std::string, Variant> fields_{};
        uint8_t temp_buf_[1024]{};
        int offset_ = 0;
    };

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const uint32_t sn, const uint16_t app_idx,
                   const std::string &suffix,
                   const T &default_value) {
        try {
            return std::get<T>(data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const uint32_t sn, const std::string &suffix,
                   const T &default_value) {
        try {
            return std::get<T>(data.get(fmt::format("sn{}_{}", sn, suffix)));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const std::string &whole_key, const T &default_value) {
        try {
            return std::get<T>(data.get(whole_key));
        } catch (const std::runtime_error &) {
            return default_value;
        }
    }

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const uint32_t sn, const uint16_t app_idx,
                   const std::string &suffix) {
        return std::get<T>(data.get(fmt::format("sn{}_app_{}_{}", sn, app_idx, suffix)));
    }

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const uint32_t sn, const std::string &suffix) {
        return std::get<T>(data.get(fmt::format("sn{}_{}", sn, suffix)));
    }

    template<typename T>
    T get_field_as(const ConfigurationParser &data, const std::string &whole_key) {
        return std::get<T>(data.get(whole_key));
    }

    ConfigurationParser *get_configuration_data();
}

#endif //CONFIGURATION_PARSER_H