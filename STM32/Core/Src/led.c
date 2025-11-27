#include "led.h"


void led_set_status(uint8_t status) {
    // Set the GPIO pins according to the status bits
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (status & 0b100)); // Blue
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (status & 0b010)); // Yellow
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (status & 0b001)); // Red
}