#include "spi_utils.h"

#include <string.h>

HAL_StatusTypeDef spi_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                           uint8_t addr, uint8_t *buffer, uint8_t len) {
  (void)hspi;
  (void)GPIOx;
  (void)GPIO_Pin;
  (void)addr;

  if (buffer != NULL && len != 0U) {
    memset(buffer, 0, len);
  }
  return HAL_OK;
}

HAL_StatusTypeDef spi_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                            uint8_t addr, uint8_t data) {
  (void)hspi;
  (void)GPIOx;
  (void)GPIO_Pin;
  (void)addr;
  (void)data;
  return HAL_OK;
}

HAL_StatusTypeDef spi_burst_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                  uint8_t addr, uint8_t *data, uint8_t len) {
  (void)hspi;
  (void)GPIOx;
  (void)GPIO_Pin;
  (void)addr;
  (void)data;
  (void)len;
  return HAL_OK;
}
