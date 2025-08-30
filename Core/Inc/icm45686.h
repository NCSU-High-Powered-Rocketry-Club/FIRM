/*
 * icm45686.h
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#pragma once
#include "usb_print_debug.h"

typedef enum IREGMap {
	IMEM_SRAM, IPREG_BAR, IPREG_SYS1, IPREG_SYS2, IPREG_TOP1
} IREGMap_t;

int imu_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);
void spi_ireg_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin, IREGMap_t register_map, uint16_t ireg_addr,
		uint8_t *result);
void spi_ireg_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin, IREGMap_t register_map, uint8_t addr, uint8_t data);

