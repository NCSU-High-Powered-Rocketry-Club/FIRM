/*
 * adxl371.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Adapted by ERM from code written by Wlsan
 */

#pragma once
#include "adxl371_packet.h"
#include "usb_print_debug.h"
#include <math.h>
#include <stdbool.h>

/**
 * @brief ensures SPI read/write is working to the ADXL371, and configures register settings.
 *
 * @note Sets ODR to 1280, data ready to INT1 (Active Low), Disables FIFO, disables High and Low
 * pass Filters,  - Turns off oustosleep and linkloop, sets chip to low noise operation.
 *
 * @param hspi pointer to the SPI channel that the ADXL371 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int adxl371_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

/**
 * @brief reads the acceleration measurements from the ADXL371
 *
 * @note used a single read statement to batch read from xdata_h to zdata_l (0x08 to 0x0D)
 *
 * @param packet pointer to the accelerometer packet where the data will be stored
 * @retval 0 upon success, 1 if no new data is ready yet
 */
int adxl371_read_data(ADXL371Packet_t *packet);
