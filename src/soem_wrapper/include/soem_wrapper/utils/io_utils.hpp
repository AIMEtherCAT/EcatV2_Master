//
// Created by hang on 25-5-5.
//

#ifndef IOUTILS_H
#define IOUTILS_H

#include "cstdint"

namespace aim::io::little_endian {
    float read_float16(const uint8_t *buf, int *offset);

    float read_float(const uint8_t *buf, int *offset);

    uint8_t read_uint8(const uint8_t *buf, int *offset);

    int16_t read_int16(const uint8_t *buf, int *offset);

    uint16_t read_uint16(const uint8_t *buf, int *offset);

    uint32_t read_uint32(const uint8_t *buf, int *offset);

    int32_t read_int32(const uint8_t *buf, int *offset);

    void write_uint8(uint8_t value, uint8_t *buf, int *offset);

    void write_int8(int8_t value, uint8_t *buf, int *offset);

    void write_float(float value, uint8_t *buf, int *offset);

    void write_int16(int16_t value, uint8_t *buf, int *offset);

    void write_int32(int32_t value, uint8_t *buf, int *offset);

    void write_uint16(uint16_t value, uint8_t *buf, int *offset);

    void write_uint32(uint32_t value, uint8_t *buf, int *offset);
}

#endif //IOUTILS_H