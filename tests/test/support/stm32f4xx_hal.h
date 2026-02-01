#ifndef STM32F4xx_HAL_H
#define STM32F4xx_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t MODER;
    uint32_t OTYPER;
    uint32_t OSPEEDR;
    uint32_t PUPDR;
    uint32_t IDR;
    uint32_t ODR;
} GPIO_TypeDef;

typedef uint32_t GPIO_PinState;

#define GPIO_PIN_RESET ((GPIO_PinState)0u)
#define GPIO_PIN_SET   ((GPIO_PinState)1u)

typedef struct {
    uint32_t Instance; 
} DMA_HandleTypeDef;

typedef struct {
    uint32_t Instance;
} SPI_HandleTypeDef;

extern GPIO_TypeDef* GPIOA;
extern GPIO_TypeDef* GPIOB;
extern GPIO_TypeDef* GPIOC;

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void HAL_Delay(uint32_t Delay);
int HAL_NVIC_SystemReset(void);

typedef enum {
  HAL_DMA_STATE_RESET = 0x00U,
  HAL_DMA_STATE_READY = 0x01U,
  HAL_DMA_STATE_BUSY = 0x02U,
  HAL_DMA_STATE_TIMEOUT = 0x03U,
  HAL_DMA_STATE_ERROR = 0x04U,
  HAL_DMA_STATE_ABORT = 0x05U
} HAL_DMA_StateTypeDef;

HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);

#endif