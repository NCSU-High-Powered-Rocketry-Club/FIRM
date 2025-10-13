/*
 * bmp581.h
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
 * @param packet pointer to the BMP packet where the data will be stored
 * @retval 0 upon success, 1 if no new data is ready yet
 */
int bmp_read_data(BMPPacket_t* packet);

/**
 * @brief gets the scale factor of the temperature readings to convert to celcius.
 * 
 * @retval float value to divide binary data by to get temperature in celcius
 */
float bmp581_get_temp_scale_factor(void);

/**
 * @brief gets the scale factor of the pressure readings to convert to pascals.
 * 
 * @retval float value to divide binary data by to get pressure in pascals
 */
float bmp581_get_pressure_scale_factor(void);