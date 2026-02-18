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
#include "mmc5983ma_packet.h"

/**
 * @brief ensures SPI read/write is working to the MMC5983MA, and configures register settings
 *
 * @param hspi pointer to the SPI channel that the MMC5983MA is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int mmc5983ma_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

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

/**
 * @brief sets the SPI settings for the MMC5983MA
 * 
 * @retval None
 */
void set_spi_mmc(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);