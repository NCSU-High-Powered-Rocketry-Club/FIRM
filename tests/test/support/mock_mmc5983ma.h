#pragma once
#include <stdint.h>

typedef struct {
    uint8_t mag_x_msb;
    uint8_t mag_x_mid;
    uint8_t mag_y_msb;
    uint8_t mag_y_mid;
    uint8_t mag_z_msb;
    uint8_t mag_z_mid;
    uint8_t mag_xyz_lsb;
} MMC5983MAPacket_t;