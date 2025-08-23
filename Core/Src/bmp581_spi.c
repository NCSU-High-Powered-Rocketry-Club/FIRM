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
const uint8_t bmp581_reg_cmd = 0x7E;
const uint8_t bmp581_reg_status = 0x28;
const uint8_t bmp581_reg_int_status = 0x27;
const uint8_t bmp581_reg_osr_config = 0x36;
const uint8_t bmp581_reg_fifo_sel = 0x18;
const uint8_t bmp581_reg_int_config = 0x14;
const uint8_t bmp581_reg_int_source = 0x15;
const uint8_t bmp581_reg_ord_config = 0x37;


int bmp_init(SPI_HandleTypeDef *hspi) {
	// drive chip select pin high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(4); // startup time from power-on to configuration change

	uint8_t result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1); // dummy read, discard info
	HAL_Delay(1);
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_cmd, 0b10110110); // do a soft-reset of the sensor's settings
	HAL_Delay(3); // delay needed after soft reset

	// do another dummy read
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);
	// verify chip ID read works
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);
	if (result != 0x50) {
		serialPrintStr("chip ID error");
		Error_Handler();
	}

	// verify device is ready to be configured
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_status, &result, 1);
	if (result != 0x02) {
		serialPrintStr("device not ready to be configured");
		Error_Handler();
	}

	// verify software reset is recognized as complete by the interrupt status register
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_status, &result, 1);
	if ((result & 0x10) == 0) { // check that bit 4 (POR) is 1
		serialPrintStr("software reset not working");
		Error_Handler();
	}

	// enable pressure measurements, sets 1x over-sampling (no OSR) for pressure and temperature.
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_osr_config, 0b01000000);
	// disable FIFO buffer
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_fifo_sel, 0b00000000);
	// enable interrupt pin, set to active-low, open-drain, latched mode
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_config, 0b00111101);
	// set the source of the interrupt signal to be on data-ready
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_source, 0b00000001);
	// disable deep-sleep, set to max ODR, set to continuous mode
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_ord_config, 0b10000011);
	// continuous mode actually ignores the ODR bits that were set, and uses the OSR to determine
	// the ODR (498hz with 1x OSR)
	return 0;
}

int bmp_read(SPI_HandleTypeDef *hspi) {
	uint8_t result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);

	serialPrintlnInt(result);
	return 0;
}



