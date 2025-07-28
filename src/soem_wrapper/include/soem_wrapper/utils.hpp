//
// Created by hang on 25-5-5.
//

#ifndef UTILS_H
#define UTILS_H

#include "stdint.h"
#include "cstring"
#include "cmath"

inline float get_percentage(const uint16_t raw_data) {
    const float target_spd = static_cast<float>(raw_data - 1024);
    if (target_spd >= -5 && target_spd <= 5) {
        return 0.0f;
    }

    return target_spd / 660.0f;
}

inline int32_t read_int32(const uint8_t* t, int* offset) {
    int32_t res;
    memcpy(&res, t + *offset, 4);
    *offset += 4;
    return res;
}

inline uint32_t read_uint32(const uint8_t* t, int* offset) {
    uint32_t res;
    memcpy(&res, t + *offset, 4);
    *offset += 4;
    return res;
}

inline float read_float(const uint8_t* t, int* offset) {
    float res;
    memcpy(&res, t + *offset, 4);
    *offset += 4;
    return res;
}

inline int16_t read_int16(const uint8_t* t, int* offset) {
    int16_t res;
    memcpy(&res, t + *offset, 2);
    *offset += 2;
    return res;
}

inline uint16_t read_uint16(const uint8_t* t, int* offset) {
    uint16_t res;
    memcpy(&res, t + *offset, 2);
    *offset += 2;
    return res;
}

inline int8_t read_int8(const uint8_t* t, int* offset) {
    const int8_t res = static_cast<int8_t>(*(t + *offset));
    *offset += 1;
    return res;
}

inline uint8_t read_uint8(const uint8_t* t, int* offset) {
    const uint8_t res = *(t + *offset);
    *offset += 1;
    return res;
}


inline void write_int32(const int32_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 4);
    *offset += 4;
}

inline void write_uint32(const uint32_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 4);
    *offset += 4;
}

inline void write_float(const float value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 4);
    *offset += 4;
}

inline void write_int16(const int16_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 2);
    *offset += 2;
}

inline void write_uint16(const uint16_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 2);
    *offset += 2;
}

inline void write_int8(const int8_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 1);
    *offset += 1;
}

inline void write_uint8(const uint8_t value, uint8_t* t, int* offset) {
    memcpy(t + *offset, &value, 1);
    *offset += 1;
}

inline float read_float16(const uint8_t *t, int *offset) {
    const uint16_t h = read_uint16(t, offset);
    const uint16_t h_exp = (h & 0x7C00) >> 10;
    const uint16_t h_frac = h & 0x03FF;
    const int32_t sign = h & 0x8000 ? -1 : 1;

    if (h_exp == 0) {
        if (h_frac == 0) {
            return sign * 0.0f;
        }
        return sign * powf(2, -14) * (h_frac / 1024.0f);
    }
    if (h_exp == 0x1F) {
        return h_frac ? NAN : sign > 0 ? INFINITY : -INFINITY;
    }
    return sign * powf(2, h_exp - 15) * (1.0f + h_frac / 1024.0f);
}

#endif //UTILS_H
