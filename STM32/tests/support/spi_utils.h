#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef spi_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                           uint8_t addr, uint8_t *buffer, uint8_t len);

HAL_StatusTypeDef spi_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                            uint8_t addr, uint8_t data);

HAL_StatusTypeDef spi_burst_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                  uint8_t addr, uint8_t *data, uint8_t len);
