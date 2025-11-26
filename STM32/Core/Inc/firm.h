#pragma once

#include <stdbool.h>
#include "stm32f4xx_hal.h"


// These flags can be changed at any time from the interrupts. When they are set
// to true, it means that the corresponding sensor has new data ready to be read.
extern volatile bool bmp581_has_new_data;
extern volatile bool icm45686_has_new_data;
extern volatile bool mmc5983ma_has_new_data;


extern struct SPIHandles {
    SPI_HandleTypeDef* hspi1;
    SPI_HandleTypeDef* hspi2;
    SPI_HandleTypeDef* hspi3;
} spi_handles;

extern struct I2CHandles {
    I2C_HandleTypeDef* hi2c1;
    I2C_HandleTypeDef* hi2c2;
} i2c_handles;

extern struct DMAHandles {
    DMA_HandleTypeDef* hdma_sdio_rx;
    DMA_HandleTypeDef* hdma_sdio_tx;
} dma_handles;


void initialize_firm(struct SPIHandles* spi_handles, struct I2CHandles* i2c_handles, struct DMAHandles* dma_handles);

void loop_firm(void);