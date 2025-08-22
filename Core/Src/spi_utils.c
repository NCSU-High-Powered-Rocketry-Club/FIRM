/*
 * spi_utils.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "spi_utils.h"

void spi_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr, uint8_t *buffer, uint8_t len) {
	addr |= 0x80;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // pull CS pin low
	HAL_SPI_Transmit(hspi, &addr, 1, 100); // send the address that you want data from
	HAL_SPI_Receive(hspi, buffer, len, 100); //read data and store into buffer
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}


void spi_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr, uint8_t data) {
	uint8_t tx_buffer[2];
	tx_buffer[0] = addr;
	tx_buffer[1] = data;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, tx_buffer, 2, 100);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

