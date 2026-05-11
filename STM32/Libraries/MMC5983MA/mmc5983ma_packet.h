#pragma once

#include <stdint.h>

/**
 * @brief the raw magnetic field bytes for the MMC5983MA.
 */
typedef struct {
  uint8_t mag_x_msb;
  uint8_t mag_x_mid;
  uint8_t mag_y_msb;
  uint8_t mag_y_mid;
  uint8_t mag_z_msb;
  uint8_t mag_z_mid;
  uint8_t mag_xyz_lsb;
} MMC5983MARawData_t;

/**
 * @brief MMC5983MA magnetic field data as floats, rotated to board-frame.
 * @note Calibration is applied.
 */
typedef struct {
  float magnetic_field_x_microteslas;
  float magnetic_field_y_microteslas;
  float magnetic_field_z_microteslas;
} MMC5983MABoardReading_t;