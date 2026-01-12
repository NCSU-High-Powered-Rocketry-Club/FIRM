#pragma once
#include <stdint.h>
#include "stm32_hal_stubs.h"

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
