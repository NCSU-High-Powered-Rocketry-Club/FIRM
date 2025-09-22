/*
 * i2c_utils.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "i2c_utils.h"

int i2c_read(I2C_HandleTypeDef* hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t* buffer,
             uint8_t len) {
    // Write the register address, then read the data
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_addr << 1), (uint16_t)reg_addr,
                                                I2C_MEMADD_SIZE_8BIT, buffer, len, 100);
    return status;
}

int i2c_write(I2C_HandleTypeDef* hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        hi2c, (uint16_t)(dev_addr << 1), (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return status;
}
