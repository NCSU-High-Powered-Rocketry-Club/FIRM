#pragma once

#include <stdint.h>

/**
 * @brief the raw acceleration bytes for the ADXL371.
 */
typedef struct {
  uint8_t accX_H;
  uint8_t accX_L;
  uint8_t accY_H;
  uint8_t accY_L;
  uint8_t accZ_H;
  uint8_t accZ_L;
} ADXL371RawData_t;

/**
 * @brief ADXL371 acceleration data as floats, rotated to board-frame.
 * @note Calibration is applied
 */
typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
} ADXL371BoardReading_t;