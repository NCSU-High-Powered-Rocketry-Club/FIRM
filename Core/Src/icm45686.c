/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "spi_utils.h"

const uint8_t who_am_i = 0x72;
const uint8_t reg_misc2 = 0x7F;

int imu_init(SPI_HandleTypeDef *hspi) {
	HAL_Delay(2); // 3ms delay to allow device to power on

	// Do a dummy read
	uint8_t result = 0;
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);

	// Verify chip ID is 0xE9
	result = 0;
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	//issues a soft reset
	spi_write(hspi, GPIOB, GPIO_PIN_9, reg_misc2, 0b00000010);
	HAL_Delay(2); // 3ms delay to allow device to finish reset

	//Do another dummy read
	result = 0;
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);
	// Verify chip ID is 0xE9
	result = 0;
	spi_read(hspi, GPIOB, GPIO_PIN_9, who_am_i, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	//Check bit 1 (soft reset bit) is set back to 0
	spi_read(hspi, GPIOB, GPIO_PIN_9, reg_misc2, &result, 1);
	if ((result & 0x02) != 0) {
		serialPrintStr("icm45686 software reset failed");
		return 1;
	}

	return 0;
}
