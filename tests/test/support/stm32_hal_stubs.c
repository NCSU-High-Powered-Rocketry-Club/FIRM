#include "stm32_hal_stubs.h"
GPIO_TypeDef gpioa_instance;
GPIO_TypeDef* GPIOA = &gpioa_instance;

// This reboots the system normally but in tests we just return 0
int HAL_NVIC_SystemReset(void) {
    return 0;
}
