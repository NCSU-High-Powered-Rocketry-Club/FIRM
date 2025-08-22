/*
 * spi_utils.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */

#pragma once
#include <stdint.h>
#include "usb_device.h"


void spi_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr, uint8_t *buffer, uint8_t len);
void spi_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr, uint8_t data);
