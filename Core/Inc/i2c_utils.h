/*
 * i2c_utils.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * Reads data from a sensor register over I2C
 * @param hi2c pointer to the I2C channel
 * @param dev_addr the 8-bit I2C address of the device
 * @param reg_addr the register address to read from
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 */
void i2c_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t len);

/**
 * Writes 1 byte of data to a sensor register over I2C
 * @param hi2c pointer to the I2C channel
 * @param dev_addr the 8-bit I2C address of the device
 * @param reg_addr the register address to write to
 * @param data the data to write
 */
void i2c_write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
