#pragma once

#include <stdint.h>

/**
 * @brief data packet for the BMP581 temperature and pressure data.
 * @note the order is determined by register order in the datasheet, starting at 0x1D.
 */
typedef struct {
  uint8_t temp_xlsb;
  uint8_t temp_lsb;
  uint8_t temp_msb;
  uint8_t pressure_xlsb;
  uint8_t pressure_lsb;
  uint8_t pressure_msb;
} BMP581Packet_t;
