#pragma once
#include "usb_print_debug.h"
#include "spi_utils.h"

/**
 * @brief ensures SPI read/write is working to the W25Q128JVS, and configures register settings
 *
 * @param hspi pointer to the SPI channel that the W25Q128JVS is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int w25q128jvs_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);