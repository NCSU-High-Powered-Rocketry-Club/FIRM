/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "usb_print_debug.h"
#include <stdbool.h>
#include <stdint.h>
#include <spi_utils.h>

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
 * @brief ensures SPI read/write is working to the MMC9583MA, and configures register settings
 *
 * @param hspi pointer to the SPI channel that the MMC9583MA is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int mmc5983ma_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads data from the MMC5983MA
 *
 * @param packet pointer to the Magnetometer packet where the data will be stored
 * @param flip counter that must be incremented to determine which read cycle will flip polarity
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_read_data(MMC5983MAPacket_t* packet, uint8_t* flip);

/**
 * @brief gets the scale factor of the magnetometer readings to convert to microteslas.
 * @note this value is the same scale factor used for all axes of the magnetometer.
 * 
 * @retval float value to divide binary data by to get magnetic field in microteslas.
 */
float mmc5983ma_get_magnetic_field_scale_factor(void);
