#pragma once

/**
 * @brief Collection of all data (and the timestamp)
 */
typedef struct {
  double timestamp_seconds;
  float temperature_celsius;
  float pressure_pascals;
  float raw_acceleration_x_gs;
  float raw_acceleration_y_gs;
  float raw_acceleration_z_gs;
  float raw_angular_rate_x_dps;
  float raw_angular_rate_y_dps;
  float raw_angular_rate_z_dps;
  float magnetic_field_x_microteslas;
  float magnetic_field_y_microteslas;
  float magnetic_field_z_microteslas;
  float high_g_accel_x_gs;
  float high_g_accel_y_gs;
  float high_g_accel_z_gs;
  float est_position_z_meters;
  float est_velocity_z_meters_per_s;
  float est_quaternion_w;
  float est_quaternion_x;
  float est_quaternion_y;
  float est_quaternion_z;
} DataPacket_t;