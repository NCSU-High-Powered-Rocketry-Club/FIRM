#pragma once
#include "settings.h"
#include <bmp581_packet.h>
#include <icm45686_packet.h>
#include <mmc5983ma_packet.h>

typedef struct {
  uint8_t timestamp[4];
  union {
    BMP581Packet_t bmp581_packet;
    ICM45686Packet_t icm45686_packet;
    MMC5983MAPacket_t mmc5983ma_packet;
  } packet;
} SensorPacket;

/**
 * @brief Data packet with timestamp
 */
typedef struct {
  double timestamp_seconds;
  float temperature_celsius;
  float pressure_pascals;
  float raw_acceleration_x_gs;
  float raw_acceleration_y_gs;
  float raw_acceleration_z_gs;
  float raw_angular_rate_x_deg_per_s;
  float raw_angular_rate_y_deg_per_s;
  float raw_angular_rate_z_deg_per_s;
  float magnetic_field_x_microteslas;
  float magnetic_field_y_microteslas;
  float magnetic_field_z_microteslas;
  float est_position_x_meters;
  float est_position_y_meters;
  float est_position_z_meters;
  float est_velocity_x_meters_per_s;
  float est_velocity_y_meters_per_s;
  float est_velocity_z_meters_per_s;
  float est_acceleration_x_gs;
  float est_acceleration_y_gs;
  float est_acceleration_z_gs;
  float est_angular_rate_x_rad_per_s;
  float est_angular_rate_y_rad_per_s;
  float est_angular_rate_z_rad_per_s;
  float est_quaternion_w;
  float est_quaternion_x;
  float est_quaternion_y;
  float est_quaternion_z;
} DataPacket;

/**
 * @brief Converts raw BMP581 pressure sensor data to SI units
 * @note Temperature converted to Celcius, pressure converted to Pascals
 *
 * @param packet Pointer to a SensorPacket structure containing raw bmp581 sensor data
 * @param result_packet Pointer to a DataPacket where the result of the preprocessor
 *                      will be stored
 */
void bmp581_convert_packet(SensorPacket *packet, DataPacket *result_packet);

/**
 * @brief Converts raw MMC5983MA magnetometer data to SI units
 * @note Magnetic field data converted to microtesla
 *
 * @param packet Pointer to an SensorPacket structure containing raw mmc5983ma sensor data
 * @param result_packet Pointer to a DataPacket where the result of the preprocessor
 *                      will be stored
 */
void mmc5983ma_convert_packet(SensorPacket *packet, DataPacket *result_packet);

/**
 * @brief Converts raw ICM45686 data to SI units
 *
 * @note Acceleration converted to g's, angular rate converted to radians per second
 *
 * @param packet Pointer to an ICM45686Packet_t structure containing raw icm45686 sensor data
 * @param result_packet Pointer to a DataPacket where the result of the preprocessor
 *                      will be stored
 */
void icm45686_convert_packet(SensorPacket *packet, DataPacket *result_packet);
