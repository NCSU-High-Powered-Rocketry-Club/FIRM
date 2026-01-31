#pragma once
#include <bmp581.h>
#include <mmc5983ma.h>
#include <icm45686.h>
#include <ADXL371.h>


/**
 * @brief Calibrated data with timestamp
 */
typedef struct {

    float temperature;
    float pressure;
    float accel_x;
    float accel_y;
    float accel_z;
    float adxl_accel_x;
    float adxl_accel_y;
    float adxl_accel_z;
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float magnetic_field_x;
    float magnetic_field_y;
    float magnetic_field_z;
    double timestamp_sec;
} CalibratedDataPacket_t;


/**
 * @brief Converts raw BMP581 pressure sensor data to SI units
 * @note Temperature converted to Celcius, pressure converted to Pascals
 * 
 * @param packet Pointer to a BMP581Packet_t structure containing raw sensor data
 * @param result_packet Pointer to a CalibratedDataPacket where the result of the preprocessor
 *                      will be stored
 */
void bmp581_convert_packet(BMP581Packet_t *packet, CalibratedDataPacket_t *result_packet);

/**
 * @brief Converts raw MMC5983MA magnetometer data to SI units
 * @note Magnetic field data converted to microtesla
 * 
 * @param packet Pointer to an MMC5983MAPacket_t structure containing raw sensor data
 * @param result_packet Pointer to a CalibratedDataPacket where the result of the preprocessor
 *                      will be stored
 */
void mmc5983ma_convert_packet(MMC5983MAPacket_t *packet, CalibratedDataPacket_t *result_packet);

/**
 * @brief Converts raw ICM45686 data to SI units
 * 
 * @note Acceleration converted to g's, angular rate converted to radians per second
 * 
 * @param packet Pointer to an ICM45686Packet_t structure containing raw sensor data
 * @param result_packet Pointer to a CalibratedDataPacket where the result of the preprocessor
 *                      will be stored
 */
void icm45686_convert_packet(ICM45686Packet_t *packet, CalibratedDataPacket_t *result_packet);

