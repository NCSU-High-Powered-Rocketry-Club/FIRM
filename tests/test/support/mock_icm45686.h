#pragma once
#include <stdint.h>

typedef struct {
    uint8_t accX_H;
    uint8_t accX_L;
    uint8_t accY_H;
    uint8_t accY_L;
    uint8_t accZ_H;
    uint8_t accZ_L;
    uint8_t gyroX_H;
    uint8_t gyroX_L;
    uint8_t gyroY_H;
    uint8_t gyroY_L;
    uint8_t gyroZ_H;
    uint8_t gyroZ_L;
    uint8_t x_vals_lsb;
    uint8_t y_vals_lsb;
    uint8_t z_vals_lsb;
} ICM45686Packet_t;