/*
 * bmp581_spi.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once
#include "usb_print_debug.h"

/**
 * @brief ensures SPI read/write is working to the BMP581, and configures register settings
 *
 * @param hspi pointer to the SPI channel that the BMP581 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int bmp_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads the pressure and temperature measurements from the BMP581
 *
 * @param hspi pointer to the SPI channel that the BMP581 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success, 1 if no new data is ready yet
 */
int bmp_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);
