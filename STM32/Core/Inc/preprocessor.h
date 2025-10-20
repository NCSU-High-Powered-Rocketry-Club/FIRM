#pragma once

#include "bmp581.h"
#include "mmc5983ma.h"
#include "icm45686.h"

typedef struct {
    float temperature;
    float pressure;
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float magnetic_field_x;
    float magnetic_field_y;
    float magnetic_field_z;
} CalibratedDataPacket_t;

void bmp581_convert_packet(BMP581Packet_t packet);
void mmc5983_convert_packet(MMC5983MAPacket_t packet);
void imu_convert_packet(ICM45686Packet_t packet);
