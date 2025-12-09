#pragma once

#include <stdbool.h>
#include "stm32f4xx_hal.h"


// These flags can be changed at any time from the interrupts. When they are set
// to true, it means that the corresponding sensor has new data ready to be read.
extern volatile bool bmp581_has_new_data;
extern volatile bool icm45686_has_new_data;
extern volatile bool mmc5983ma_has_new_data;


// Struct to contain all SPI handles for the firm initialization function
typedef struct {
    SPI_HandleTypeDef* hspi1;
    SPI_HandleTypeDef* hspi2;
    SPI_HandleTypeDef* hspi3;
} SPIHandles;

// Struct to contain all I2C handles for the firm initialization function
typedef struct {
    I2C_HandleTypeDef* hi2c1;
    I2C_HandleTypeDef* hi2c2;
} I2CHandles;


// Struct to contain all DMA handles for the firm initialization function
 typedef struct {
    DMA_HandleTypeDef* hdma_sdio_rx;
    DMA_HandleTypeDef* hdma_sdio_tx;
} DMAHandles;


/**
 * @brief Initializes firm, including all sensors and the logger
 *
 * @param spi_handles struct containing all SPI handles, from the HAL
 * @param dma_handles struct containing all DMA handles, from the HAL
 * @retval Whether FIRM successfully initialized, 0 on successful write
 */
int initialize_firm(SPIHandles* spi_handles, DMAHandles* dma_handles);

/**
 * @brief The main loop which checks if any of the sensors have new data, reads it, logs it, and
 *        sends it over USB if connected.
 */
void loop_firm(void);