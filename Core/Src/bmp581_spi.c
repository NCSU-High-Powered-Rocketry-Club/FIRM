/*
 * bmp581_spi.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "bmp581_spi.h"
#include "spi_utils.h"
#include <stdint.h>

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
const uint8_t bmp581_reg_temp_data_xlsb = 0x1D;


int bmp_init(SPI_HandleTypeDef *hspi) {
	// drive chip select pin high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(4); // from data sheet: startup time from power-on to configuration change

	uint8_t result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1); // dummy read, discard info
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_cmd, 0b10110110); // do a soft-reset of the sensor's settings
	HAL_Delay(3); // from data sheet: delay needed after soft reset

	// do another dummy read
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);
	// verify chip ID read works
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_chip_id, &result, 1);
	if (result != 0x50) {
		serialPrintStr("BMP581 chip ID failed to read");
		return 1;
	}

	// verify device is ready to be configured
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_status, &result, 1);
	if (result != 0x02) {
		serialPrintStr("BMP581 not ready to be configured");
		return 1;
	}

	// verify software reset is recognized as complete by the interrupt status register
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_status, &result, 1);
	if ((result & 0x10) == 0) { // check that bit 4 (POR) is 1
		serialPrintStr("BMP581 software reset failed");
		return 1;
	}

	// enable pressure measurements, sets 1x over-sampling (no OSR) for pressure and temperature.
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_osr_config, 0b01000000);
	// disable FIFO buffer
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_fifo_sel, 0b00000000);
	// enable interrupt pin, set to active-low, open-drain, latched mode
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_config, 0b00111001);
	// set the source of the interrupt signal to be on data-ready
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_source, 0b00000001);
	// disable deep-sleep, set to max ODR, set to continuous mode
	uint8_t intended_ord_config_setting = 0b10000011;
	spi_write(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_ord_config, intended_ord_config_setting);
	// continuous mode actually ignores the ODR bits that were set, and uses the OSR to determine
	// the ODR (498hz with 1x OSR)

	// verify one of the writes works, error otherwise
	result = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_ord_config, &result, 1);
	if (result != intended_ord_config_setting) {
		serialPrintStr("BMP581 configuration write failed");
		return 1;
	}
	return 0;
}

int bmp_read(SPI_HandleTypeDef *hspi) {
	// clear interrupt (pulls interrupt back up high) and verify new data is ready
	uint8_t data_ready = 0;
	spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_int_status, &data_ready, 1);
	if (data_ready & 0x01) { // bit 0 (LSB) will be 1 if new data is ready
		// temperature and pressure are both 24 bit values, with the data in 3 registers each
		// burst read 6 registers starting from XLSB of temp, to MSB of pressure (0x1D -> 0x22)
		uint8_t raw_data[6];
		spi_read(hspi, GPIOC, GPIO_PIN_2, bmp581_reg_temp_data_xlsb, raw_data, 6);
		// bit shift the raw data, MSB shifts 16 bits left, LSB 8 bits left, and XLSB rightmost
		int32_t raw_temp = ((int32_t)raw_data[2] << 16) | ((int32_t)raw_data[1] << 8) | raw_data[0];
		uint32_t raw_pres = ((uint32_t)raw_data[5] << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[3];
		// datasheet instructs to divide raw temperature by 2^16 to get value in celcius, and
		// divide raw pressure by 2^6 to get value in Pascals
		float temp = raw_temp / 65536.0f;
		float pres = raw_pres / 64.0f;
		serialPrintFloat(temp);
		serialPrintFloat(pres);
		return 0;
	}
	return 1;
}



