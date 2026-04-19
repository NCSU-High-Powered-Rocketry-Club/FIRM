#pragma once

#include <stdint.h>

/**
 * @brief the raw pressure and temperature bytes for the BMP581.
 */
typedef struct {
  uint8_t temp_xlsb;
  uint8_t temp_lsb;
  uint8_t temp_msb;
  uint8_t pressure_xlsb;
  uint8_t pressure_lsb;
  uint8_t pressure_msb;
} BMP581RawData_t;

/**
 * @brief BMP581 temperature/pressure data as floats.
 */
typedef struct {
  float temperature_celsius;
  float pressure_pa;
} BMP581BoardReading_t;