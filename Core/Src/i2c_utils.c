/*
 * i2c_utils.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "i2c_utils.h"

void i2c_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t len) {
    // Write the register address, then read the data
    HAL_I2C_Mem_Read(hi2c, dev_addr, (uint16_t) (reg_addr << 1), I2C_MEMADD_SIZE_8BIT, buffer, len, 100);
}

void i2c_write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, dev_addr, (uint16_t) (reg_addr << 1), I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}
