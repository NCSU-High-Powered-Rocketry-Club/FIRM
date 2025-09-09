/*
 * spi_utils.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */

#pragma once
#include "usb_device.h"
#include <stdint.h>

/**
 * Reads data from a sensor address
 * @param hspi pointer to the SPI channel
 * @param GPIOx the channel of the CS GPIO pin
 * @param GPIO_Pin the pin number of the CS GPIO pin
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 */
void spi_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr,
              uint8_t* buffer, uint8_t len);

/**
 * Writes 1 byte of data to a sensor address
 * @param hspi pointer to the SPI channel
 * @param GPIOx the channel of the CS GPIO pin
 * @param GPIO_Pin the pin number of the CS GPIO pin
 * @param addr the address of the register
 * @param data the data to write to the register
 */
void spi_write(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr,
               uint8_t data);

/**
 * Writes 1 or more bytes of data to sequential sensor addresses
 * @param hspi pointer to the SPI channel
 * @param GPIOx the channel of the CS GPIO pin
 * @param GPIO_Pin the pin number of the CS GPIO pin
 * @param addr the address of the first register in the burst write
 * @param data the data to write to the registers
 * @param len the number of bytes to write
 */
void spi_burst_write(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t addr,
                     uint8_t* data, uint8_t len);
