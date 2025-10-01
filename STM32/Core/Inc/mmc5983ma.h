/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "i2c_utils.h"
#include "packets.h"
#include "usb_print_debug.h"
#include <stdint.h>

/**
 * @brief sets up the MMC5983MA magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @ret error status, returns 0 on success, 1 on failure
 */
int mag_init(I2C_HandleTypeDef* hi2c);

/**
 * @brief reads data from the magnetometer
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param packet pointer to the MMC packet where the data will be stored
 * @param flip counter that must be incremented to determine which read cycle will flip polarity
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mag_read(I2C_HandleTypeDef* hi2c, MMCPacket_t* packet, uint8_t* flip);
