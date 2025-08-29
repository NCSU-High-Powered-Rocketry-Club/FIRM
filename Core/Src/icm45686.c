/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "spi_utils.h"

const uint8_t pwr_mgmt0 = 0x10;
const uint8_t int1_config0 = 0x16;
const uint8_t int1_config2 = 0x18;
const uint8_t int1_status0 = 0x19;
const uint8_t int1_status1 = 0x1A;
const uint8_t accel_config0 = 0x1B;
const uint8_t gyro_config0 = 0x1C;
const uint8_t ioc_pad_scenario = 0x2F;
const uint8_t ioc_pad_scenario_aux_ovrd = 0x30;
const uint8_t who_am_i = 0x72;
const uint8_t ireg_addr_15_8 = 0x7C; // used to read/write from indirect registers (IREG)
const uint8_t ireg_addr_7_0 = 0x7D; // used to read/write from indirect registers (IREG)
const uint8_t ireg_data = 0x7E;
const uint8_t reg_misc2 = 0x7F;
const uint8_t sreg_ctrl = 0x67; // IPREG_TOP1 register
const uint8_t ipreg_sys1_reg_166 = 0xA6; // IPREG_SYS1 register
const uint8_t ipreg_sys2_reg_123 = 0x7B; // IPREG_SYS2 register


int imu_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
	HAL_Delay(2); // 3ms delay to allow device to power on

	// Do a dummy read
	uint8_t result = 0;
	spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);

	// Verify chip ID is 0xE9
	result = 0;
	spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	//issues a soft reset
	spi_write(hspi, cs_channel, cs_pin, reg_misc2, 0b00000010);
	HAL_Delay(2); // 3ms delay to allow device to finish reset

	//Do another dummy read
	result = 0;
	spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
	// Verify chip ID is 0xE9
	result = 0;
	spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
	if (result != 0xE9) {
		serialPrintStr("icm45686 chip ID failed to read");
		return 1;
	}

	//Check bit 1 (soft reset bit) is set back to 0
	spi_read(hspi, cs_channel, cs_pin, reg_misc2, &result, 1);
	if ((result & 0x02) != 0) {
		serialPrintStr("icm45686 software reset failed");
		return 1;
	}

	// overrides the aux1 channel to disable it. We are only using one output channel for the IMU
	// and not connecting other sensors for the IMU to interface with, so AUX1 along with many
	// of the other fancy features on the IMU will be disabled or unused.
	spi_write(hspi, cs_channel, cs_pin, ioc_pad_scenario_aux_ovrd, 0b00000010);

	// check if the aux1 channel is disabled (bit 0 is set to 0)
	spi_read(hspi, cs_channel, cs_pin, ioc_pad_scenario, &result, 1);
//	if ((result & 0x01) != 0) {
//		serialPrintStr("AUX1 channel failed to disable");
//		return 1;
//	}

	// sets accel range to +/- 32g, and ODR to 1600hz
	spi_write(hspi, cs_channel, cs_pin, accel_config0, 0b00000101);
	// sets gyro range to 4000dps, and ODR to 1600hz
	spi_write(hspi, cs_channel, cs_pin, gyro_config0, 0b00000101);
	// sets interrupt pin to only trigger when data is ready or reset is complete
	spi_write(hspi, cs_channel, cs_pin, int1_config0, 0b10000100);
	// sets interrupt pin to push-pull, latching, and active low
	spi_write(hspi, cs_channel, cs_pin, int1_config2, 0b00000010);

	spi_ireg_read(hspi, cs_channel, cs_pin, IPREG_TOP1, (uint16_t) 0x59, &result);
	serialPrintlnInt(result);

	return 0;
}

void spi_ireg_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin, IREGMap register_map, uint16_t addr, uint8_t *result) {
	uint16_t ireg_addr = addr;
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
	ireg_regs[0] = (uint8_t)(ireg_addr >> 8);
	ireg_regs[1] = (uint8_t)(ireg_addr & 0x00FF);

	spi_burst_write(hspi, cs_channel, cs_pin, ireg_addr_15_8, ireg_regs, 2);

	spi_read(hspi, cs_channel, cs_pin, ireg_data, result, 1);


}

void spi_ireg_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin, IREGMap register_map, uint8_t addr, uint8_t data) {

}
