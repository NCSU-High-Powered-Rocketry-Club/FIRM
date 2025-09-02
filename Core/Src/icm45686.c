/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "spi_utils.h"
#include "firm_utils.h"

// honestly i think this is probably a good thing do make a preprocessor macro but probably later
// most likely don't need more digits because we only have a single-precision FPU
const float pi = 3.14159265;

const uint8_t accel_data_x_ui_msb = 0x00;
const uint8_t PWR_MGMT0 = 0x10;
const uint8_t INT1_CONFIG0 = 0x16;
const uint8_t INT1_CONFIG2 = 0x18;
const uint8_t INT1_STATUS0 = 0x19;
const uint8_t INT1_STATUS1 = 0x1A;
const uint8_t ACCEL_CONFIG0 = 0x1B;
const uint8_t GYRO_CONFIG0 = 0x1C;
const uint8_t WHO_AM_I = 0x72;
const uint8_t IREG_ADDR_15_8 = 0x7C; // used to read/write from indirect registers (IREG)
const uint8_t IREG_ADDR_7_0 = 0x7D; // used to read/write from indirect registers (IREG)
const uint8_t IREG_DATA = 0x7E;
const uint8_t REG_MISC2 = 0x7F;
const uint8_t SREG_CTRL = 0x67; // IPREG_TOP1 register
const uint8_t IPREG_SYS1_REG_166 = 0xA6; // IPREG_SYS1 register
const uint8_t IPREG_SYS2_REG_123 = 0x7B; // IPREG_SYS2 register

int imu_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin) {
	HAL_Delay(2); // 3ms delay to allow device to power on

	// Do a dummy read
	uint8_t result = 0;
	spi_read(hspi, cs_channel, cs_pin, WHO_AM_I, &result, 1);
	result = 0;
	// Verify chip ID is 0xE9
	spi_read(hspi, cs_channel, cs_pin, WHO_AM_I, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	// issues a soft reset
	spi_write(hspi, cs_channel, cs_pin, REG_MISC2, 0b00000010);
	HAL_Delay(2); // 3ms delay to allow device to finish reset
	result = 0;

	// Do another dummy read
	spi_read(hspi, cs_channel, cs_pin, WHO_AM_I, &result, 1);
	// Verify chip ID is 0xE9
	spi_read(hspi, cs_channel, cs_pin, WHO_AM_I, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	//Check bit 1 (soft reset bit) is set back to 0
	spi_read(hspi, cs_channel, cs_pin, REG_MISC2, &result, 1);
	if ((result & 0x02) != 0) {
		serialPrintStr("icm45686 software reset failed");
		return 1;
	}

	// sets accel range to +/- 32g, and ODR to 1600hz
	spi_write(hspi, cs_channel, cs_pin, ACCEL_CONFIG0, 0b00000101);
	// sets gyro range to 4000dps, and ODR to 1600hz
	spi_write(hspi, cs_channel, cs_pin, GYRO_CONFIG0, 0b00000101);
	// sets interrupt pin to only trigger when data is ready or reset is complete
	spi_write(hspi, cs_channel, cs_pin, INT1_CONFIG0, 0b10000100);
	// sets interrupt pin to push-pull, latching, and active low
	spi_write(hspi, cs_channel, cs_pin, INT1_CONFIG2, 0b00000010);

	spi_ireg_read(hspi, cs_channel, cs_pin, IPREG_TOP1, (uint16_t) 0x59,
			&result);
	serialPrintlnInt(result);

	return 0;
}

int imu_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin) {
	uint8_t data_ready = 0;
	// checking (and resetting) interrupt status
	spi_read(hspi, cs_channel, cs_pin, INT1_STATUS0, &data_ready, 1);
	if (data_ready & 0x04) { // bit 2 is data_ready flag for UI channel
		// 2 bytes for each data point, we will ignore temperature data.
		uint8_t raw_data[12];
		spi_read(hspi, cs_channel, cs_pin, accel_data_x_ui_msb, raw_data, 12);

		// data is stored in the registers as two 8-bit values in two's complement form, so they
		// must be converted to signed 16-bit integers.
		int16_t ax = twos_complement_16(raw_data[0], raw_data[1]);
		int16_t ay = twos_complement_16(raw_data[2], raw_data[3]);
		int16_t az = twos_complement_16(raw_data[4], raw_data[5]);
		int16_t gx = twos_complement_16(raw_data[6], raw_data[7]);
		int16_t gy = twos_complement_16(raw_data[8], raw_data[9]);
		int16_t gz = twos_complement_16(raw_data[10], raw_data[11]);

		// TODO: determine whether the data should be logged before or after the scale factor
		// is applied.

		// datasheet lists the scale factor for accelerometer to be 1,024 LSB/g when the range is
		// at 32g. We don't want unnecessary reads during this function, so for simplicity the
		// 32g scale factor will be hardcoded here. Later we can determine a better way to read
		// the scale factor
		ax /= 1024.0f;
		ay /= 1024.0f;
		az /= 1024.0f;
		// datasheet lists gyroscope scale factor as 8.192 LSB/(deg/s). We will also convert to
		// radians, coming out to 1474.56 / PI
		gx /= (1474.56f / pi);
		gy /= (1474.56f / pi);
		gz /= (1474.56f / pi);
		return 0;

	}
	return 1; // data was not ready, return error
}

void spi_ireg_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel,
		uint16_t cs_pin, IREGMap_t register_map, uint16_t ireg_addr,
		uint8_t *result) {
	switch (register_map) {
	case IMEM_SRAM:
		ireg_addr |= 0x0000;
		break;
	case IPREG_BAR:
		ireg_addr |= 0xA000;
		break;
	case IPREG_SYS1:
		ireg_addr |= 0xA400;
		break;
	case IPREG_SYS2:
		ireg_addr |= 0xA500;
		break;
	case IPREG_TOP1:
		ireg_addr |= 0xA200;
		break;
	default:
		return;
	}
	uint8_t ireg_regs[2];
	ireg_regs[0] = (uint8_t) (ireg_addr >> 8);
	ireg_regs[1] = (uint8_t) (ireg_addr & 0x00FF);

	// Starting the burst write from IREG_ADDR_15_8 means that IREG_ADDR_7_0 will also be
	// written to, since it is auto-incremented (that's what a SPI burst write does).
	// So here, sending 2 bytes means 2 addresses (IREG_ADDR_15_8 and IREG_ADDR_7_0) are
	// configured at once.
	spi_burst_write(hspi, cs_channel, cs_pin, IREG_ADDR_15_8, ireg_regs,
			sizeof(ireg_regs));

	// After the write is over and the CS pin is pulled high, result of the the read
	// will be stored in the IREG_DATA register. The `result` pointer will have the value:
	// NOTE: We must wait for a minimum of 4us before reading IREG_DATA:

	HAL_Delay(1);  // can only do millisecond delays, so 1 is enough

	spi_read(hspi, cs_channel, cs_pin, IREG_DATA, result, 1);

}

void spi_ireg_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel,
		uint16_t cs_pin, IREGMap_t register_map, uint16_t ireg_addr, uint8_t data) {
	//combine page with address in as 16 bits
	switch (register_map) {
	case IMEM_SRAM:
		ireg_addr |= 0x0000;
		break;
	case IPREG_BAR:
		ireg_addr |= 0xA000;
		break;
	case IPREG_SYS1:
		ireg_addr |= 0xA400;
		break;
	case IPREG_SYS2:
		ireg_addr |= 0xA500;
		break;
	case IPREG_TOP1:
		ireg_addr |= 0xA200;
		break;
	default:
		return;
	}

	// save the page, register, and data into an array
	uint8_t ireg_regs[3];
	ireg_regs[0] = (uint8_t) (ireg_addr >> 8);
	ireg_regs[1] = (uint8_t) (ireg_addr & 0x00FF);
	ireg_regs[2] = data;

	// burst write page,register, and data starting at IREG_ADDR_15_8
	spi_burst_write(hspi, cs_channel, cs_pin, IREG_ADDR_15_8, ireg_regs, 3);

}

