#pragma once
#include <stdint.h>

// TODO: add units to var names

typedef struct {
    // Units: G's
    float acc_x;
    float acc_y;
    float acc_z;
    // Units: rad/s
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMUPacket_t;

typedef struct {
    // Units: degrees Celsius
    float temperature;
    // Units: Pascals
    float pressure;
} BarometerPacket_t;

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} MagnetometerPacket_t;
