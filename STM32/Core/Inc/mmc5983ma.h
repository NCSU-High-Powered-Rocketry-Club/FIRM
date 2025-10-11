/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "packets.h"
#include "usb_print_debug.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief the I2C settings for the MMC5983MA to use when accessing device registers
 */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_addr; // 7 bit i2c address for the device
} I2CSettings;

/**
 * @brief sets up the MMC5983MA magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param device_i2c_addr the 7-bit MMC5983MA's i2c device address, default is 0x30
 * @ret error status, returns 0 on success, 1 on failure
 */
int MMC5983MA_init(I2C_HandleTypeDef* hi2c, uint8_t device_i2c_addr);

/**
 * @brief reads data from the MMC5983MA
 *
 * @param packet pointer to the Magnetometer packet where the data will be stored
 * @param flip counter that must be incremented to determine which read cycle will flip polarity
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int MMC5983MA_read_data(MagnetometerPacket_t* packet, uint8_t* flip);
