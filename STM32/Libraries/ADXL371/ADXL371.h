#pragma once
#include "usb_print_debug.h"
#include <stdbool.h>
#include <math.h>

typedef struct{
    uint8_t accX_H;
    uint8_t accX_L;
    uint8_t accY_H;
    uint8_t accY_L;
    uint8_t accZ_H;
    uint8_t accZ_L;
}ADXL371Packet_t;