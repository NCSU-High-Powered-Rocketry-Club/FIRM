/*
 * bmp581_spi.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "bmp581_spi.h"

// register for chip product ID (to test SPI transfers)
const uint8_t bmp581_reg_chip_id = 0x01;

int bmp_init() {
	// drive chip select pin high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	return 0;
}

int bmp_read(SPI_HandleTypeDef *hspi) {
	uint8_t result = 0;
	uint8_t reg = bmp581_reg_chip_id | 0x80;

	// pull CS pin low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1, 100);
	HAL_SPI_Receive(hspi, &result, 1, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);



	serialPrintlnInt(result);
	return 0;
}



