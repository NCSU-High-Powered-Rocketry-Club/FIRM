/*
 * bmp581_spi.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once
#include "packets.h"
#include "usb_print_debug.h"
#include <stdbool.h>

/**
 * @brief the SPI settings for the BMP to use when accessing device registers
 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    uint16_t cs_pin;
} BMPSPISettings;


/**
 * @brief ensures SPI read/write is working to the BMP581, and configures register settings
 *
 * @param hspi pointer to the SPI channel that the BMP581 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int bmp_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads the pressure and temperature measurements from the BMP581
 *
 * @param hspi pointer to the SPI channel that the BMP581 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @param packet pointer to the BMP packet where the data will be stored
 * @retval 0 upon success, 1 if no new data is ready yet
 */
int bmp_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
             BMPPacket_t* packet);

/**
 * @brief Starts up and resets the BMP, confirms the SPI read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
int bmp_setup_device(bool soft_reset_complete);

/**
 * @briefReads data from the BMP581 with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
HAL_StatusTypeDef bmp_spi_read(uint8_t addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the BMP581 with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
HAL_StatusTypeDef bmp_spi_write(uint8_t addr, uint8_t data);



