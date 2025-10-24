#pragma once
#include <stdint.h>

typedef struct {
  uint8_t temp_xlsb;
  uint8_t temp_lsb;
  uint8_t temp_msb;
  uint8_t pressure_xlsb;
  uint8_t pressure_lsb;
  uint8_t pressure_msb;
} BMP581Packet_t;
