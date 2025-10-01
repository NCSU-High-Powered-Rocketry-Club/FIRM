/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "packets.h"
#include "usb_print_debug.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief the SPI settings for the BMP to use when accessing device registers
 */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_addr; // 7 bit i2c address for the device
} MagI2CSettings;

/**
 * @brief sets up the MMC5983MA magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param device_i2c_addr the 7-bit MMC5983MA's i2c device address, default is 0x30
 * @ret error status, returns 0 on success, 1 on failure
 */
int mag_init(I2C_HandleTypeDef* hi2c, uint8_t device_i2c_addr);

/**
 * @brief reads data from the magnetometer
 *
 * @param packet pointer to the MMC packet where the data will be stored
 * @param flip counter that must be incremented to determine which read cycle will flip polarity
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mag_read(MMCPacket_t* packet, uint8_t* flip);

/**
 * @brief Starts up and resets the magnetometer, confirms the I2C read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
int mag_setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
HAL_StatusTypeDef mag_i2c_read(uint8_t reg_addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
HAL_StatusTypeDef mag_i2c_write(uint8_t reg_addr, uint8_t data);
