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
 * @brief sets up the MMC5983MA magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param device_i2c_addr the 7-bit MMC5983MA's i2c device address, default is 0x30
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_init(I2C_HandleTypeDef* hi2c, uint8_t device_i2c_addr);

/**
 * @brief reads data from the MMC5983MA
 *
 * @param packet pointer to the Magnetometer packet where the data will be stored
 * @param flip counter that must be incremented to determine which read cycle will flip polarity
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_read_data(MagnetometerPacket_t* packet, uint8_t* flip);

/**
 * @brief gets the scale factor of the magnetometer readings to convert to microteslas.
 * @note this value is the same scale factor used for all axes of the magnetometer.
 * 
 * @retval float value to divide binary data by to get magnetic field in microteslas.
 */
float mmc5983ma_get_magnetic_field_scale_factor(void);
