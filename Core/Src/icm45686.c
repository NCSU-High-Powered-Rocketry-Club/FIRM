/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "spi_utils.h"

const uint8_t who_am_i = 0x72;

int imu_init(SPI_HandleTypeDef *hspi) {
	HAL_Delay(2); // 3ms delay to allow device to power on

	uint8_t result = 0;
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);


	return 0;
}
