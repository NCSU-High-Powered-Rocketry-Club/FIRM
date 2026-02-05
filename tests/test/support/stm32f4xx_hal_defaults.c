#include "stm32f4xx_hal.h"

static GPIO_TypeDef struct_dummy_gpio_a;
static GPIO_TypeDef struct_dummy_gpio_b;
static GPIO_TypeDef struct_dummy_gpio_c;

// We must define GPIOA and the others because it is declared 'extern' in stm32f4xx_hal.h.
// In a real STM32, this points to a memory address. In tests, a dummy struct works.
GPIO_TypeDef* GPIOA = &struct_dummy_gpio_a;
GPIO_TypeDef* GPIOB = &struct_dummy_gpio_b;
GPIO_TypeDef* GPIOC = &struct_dummy_gpio_c;
