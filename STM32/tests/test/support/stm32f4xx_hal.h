#pragma once

#include <stdint.h>

typedef struct {
  int dummy;
} GPIO_TypeDef;

typedef enum {
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
  int dummy;
} SPI_HandleTypeDef;

void HAL_Delay(uint32_t Delay);

