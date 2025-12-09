/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "usb_print_debug.h"
#include "spi_utils.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief magnetometer data packet structur for the MMC5983MA.
 */
typedef struct {
    uint8_t mag_x_msb;
    uint8_t mag_x_mid;
    uint8_t mag_y_msb;
    uint8_t mag_y_mid;
    uint8_t mag_z_msb;
    uint8_t mag_z_mid;
    uint8_t mag_xyz_lsb;
} MMC5983MAPacket_t;

/**
 * @brief sets up the MMC5983MA magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param device_i2c_addr the 7-bit MMC5983MA's i2c device address, default is 0x30
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads data from the MMC5983MA
 *
 * @param packet pointer to the Magnetometer packet where the data will be stored
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_read_data(MMC5983MAPacket_t* packet);

/**
 * @brief gets the scale factor of the magnetometer readings to convert to microteslas.
 * @note this value is the same scale factor used for all axes of the magnetometer.
 * 
 * @retval float value to divide binary data by to get magnetic field in microteslas.
 */
float mmc5983ma_get_magnetic_field_scale_factor(void);