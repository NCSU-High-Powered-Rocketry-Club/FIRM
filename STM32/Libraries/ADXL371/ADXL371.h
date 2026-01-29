/*
 * icm45686.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Modified by ERM from code written my Wlsan
 */

#pragma once
#include "usb_print_debug.h"
#include <stdbool.h>
#include <math.h>

/**
 * @brief the SPI settings for the ADXL371 to use when accessing device registers
 */
typedef struct{
    uint8_t accX_H;
    uint8_t accX_L;
    uint8_t accY_H;
    uint8_t accY_L;
    uint8_t accZ_H;
    uint8_t accZ_L;
}ADXL371Packet_t;

