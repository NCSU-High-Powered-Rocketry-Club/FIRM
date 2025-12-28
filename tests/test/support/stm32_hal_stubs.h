#pragma once
#include <stdint.h>

typedef struct { int dummy; } SPI_HandleTypeDef;
typedef struct { int dummy; } GPIO_TypeDef;

typedef uint32_t GPIO_PinState;

#define GPIO_PIN_RESET ((GPIO_PinState)0u)
#define GPIO_PIN_SET   ((GPIO_PinState)1u)

extern GPIO_TypeDef* GPIOA;
#define GPIO_PIN_4 ((uint16_t)0x0010u)
