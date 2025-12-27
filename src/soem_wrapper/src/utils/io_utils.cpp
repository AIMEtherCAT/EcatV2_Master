//
// Created by hang on 12/26/25.
//
#include "soem_wrapper/utils/io_utils.hpp"

#include <cmath>

namespace aim::io::little_endian {
    float read_float16(const uint8_t *buf, int *offset) {
        const uint16_t h = static_cast<uint16_t>(buf[*offset]) | static_cast<uint16_t>(buf[*offset + 1]) << 8;
        *offset += 2;

        const uint16_t exp = (h & 0x7C00) >> 10;
        const uint16_t frac = h & 0x03FF;
        const int sign = h & 0x8000 ? -1 : 1;

        if (exp == 0) {
            return frac
                       ? static_cast<float>(sign) * powf(2, -14) * (static_cast<float>(frac) / 1024.0f)
                       : static_cast<float>(sign) * 0.0f;
        }
        if (exp == 0x1F) {
            return frac ? NAN : sign > 0 ? INFINITY : -INFINITY;
        }
        return static_cast<float>(sign) * powf(2, static_cast<float>(exp - 15)) * (
                   1.0f + static_cast<float>(frac) / 1024.0f);
    }

    float read_float(const uint8_t *buf, int *offset) {
        float res;
        auto *p = reinterpret_cast<uint8_t *>(&res);
        for (int i = 0; i < 4; i++) {
            p[i] = buf[*offset + i];
        }
        *offset += 4;
        return res;
    }

    uint32_t read_uint32(const uint8_t *buf, int *offset) {
        uint32_t res = 0;
        for (int i = 0; i < 4; i++) {
            res |= static_cast<uint32_t>(buf[*offset + i]) << (8 * i);
        }
        *offset += 4;
        return res;
    }

    int32_t read_int32(const uint8_t *buf, int *offset) {
        return static_cast<int32_t>(read_uint32(buf, offset));
    }

    uint16_t read_uint16(const uint8_t *buf, int *offset) {
        const uint16_t res = buf[*offset] | buf[*offset + 1] << 8;
        *offset += 2;
        return res;
    }

    int16_t read_int16(const uint8_t *buf, int *offset) {
        return static_cast<int16_t>(read_uint16(buf, offset));
    }

    uint8_t read_uint8(const uint8_t *buf, int *offset) {
        return buf[(*offset)++];
    }

    void write_uint8(const uint8_t value, uint8_t *buf, int *offset) {
        buf[(*offset)++] = value;
    }

    void write_int8(const int8_t value, uint8_t *buf, int *offset) {
        write_uint8(static_cast<uint8_t>(value), buf, offset);
    }

    void write_uint16(const uint16_t value, uint8_t *buf, int *offset) {
        buf[(*offset)++] = value & 0xFF;
        buf[(*offset)++] = value >> 8;
    }

    void write_int16(const int16_t value, uint8_t *buf, int *offset) {
        write_uint16(static_cast<uint16_t>(value), buf, offset);
    }

    void write_uint32(const uint32_t value, uint8_t *buf, int *offset) {
        for (int i = 0; i < 4; i++) {
            buf[(*offset)++] = value >> (8 * i) & 0xFF;
        }
    }

    void write_int32(const int32_t value, uint8_t *buf, int *offset) {
        write_uint32(static_cast<uint32_t>(value), buf, offset);
    }

    void write_float(float value, uint8_t *buf, int *offset) {
        const uint8_t *p = reinterpret_cast<uint8_t *>(&value);
        for (int i = 0; i < 4; i++) {
            buf[(*offset)++] = p[i];
        }
    }
}
