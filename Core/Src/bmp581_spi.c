/*
 * bmp581_spi.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "bmp581_spi.h"
#include "spi_utils.h"

// register for chip product ID (to test SPI transfers)
const uint8_t bmp581_reg_chip_id = 0x01;

int bmp_init(SPI_HandleTypeDef *hspi) {
	// drive chip select pin high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(10000);


	uint8_t result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1); // dummy read


	spi_read(hspi, GPIOC, GPIO_PIN_2, 0x37, &result, 1);
	serialPrintlnInt(result);
	HAL_Delay(10);
	spi_write(hspi, GPIOC, GPIO_PIN_2, 0x37, 0x71);
	HAL_Delay(10);
	spi_read(hspi, GPIOC, GPIO_PIN_2, 0x37, &result, 1);
	serialPrintlnInt(result);
	return 0;
}

int bmp_read(SPI_HandleTypeDef *hspi) {
	uint8_t result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);

	serialPrintlnInt(result);
	return 0;
}



