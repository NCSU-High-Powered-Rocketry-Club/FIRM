#pragma once
#include <stdint.h>
#include "mock_bmp581.h"
#include "mock_mmc5983ma.h"
#include "mock_icm45686.h"
#include "mock_dwt.h"
#include "mock_settings.h"

// For testing - we need calibrationSettings to be accessible
extern CalibrationSettings_t calibrationSettings;

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
    double timestamp_sec;
} DataPacket_t;

void bmp581_convert_packet(BMP581Packet_t *packet, DataPacket_t *result_packet);
void mmc5983ma_convert_packet(MMC5983MAPacket_t *packet, DataPacket_t *result_packet);
void icm45686_convert_packet(ICM45686Packet_t *packet, DataPacket_t *result_packet);
