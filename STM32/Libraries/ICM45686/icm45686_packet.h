#pragma once

#include <stdint.h>

/**
 * @brief Packet structure of the ICM45686 hi-res FIFO data. The temp and timestamp fields are
 *        not included.
 * @note Refer to the datasheet section 6.1 "packet structure" for information on the packet
 *       structure to see which bytes of the FIFO packet go to which data points.
 */
typedef struct {
  uint8_t accX_H;
  uint8_t accX_L;
  uint8_t accY_H;
  uint8_t accY_L;
  uint8_t accZ_H;
  uint8_t accZ_L;
  uint8_t gyroX_H;
  uint8_t gyroX_L;
  uint8_t gyroY_H;
  uint8_t gyroY_L;
  uint8_t gyroZ_H;
  uint8_t gyroZ_L;
  uint8_t x_vals_lsb;
  uint8_t y_vals_lsb;
  uint8_t z_vals_lsb;
} ICM45686Packet_t;
