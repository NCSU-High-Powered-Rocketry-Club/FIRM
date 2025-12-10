#pragma once

#include <stdint.h>

// LED status definitions
// 3-bit status codes for the 3 LEDs Blue, Yellow, and Red - in order.
typedef enum {
    FIRM_UNINITIALIZED = 0b111,
    IMU_FAIL           = 0b110,
    BMP581_FAIL        = 0b101,
    MMC5983MA_FAIL     = 0b100,
    FLASH_CHIP_FAIL    = 0b011,
    SD_CARD_FAIL       = 0b010,
    FIRM_INITIALIZED   = 0b000,
} LED_Status;

// Failed interrupt triggered LEDs:
typedef enum {
    FAILED_INTERRUPT_IMU  = 0b100,
    FAILED_INTERRUPT_BMP  = 0b010,
    FAILED_INTERRUPT_MAG  = 0b001,
} LED_Interrupt_Status;

/**
 * @brief Sets all 3 LEDs based on the given status byte.
 *
 * @param status 3-bit status code representing the state of the LEDs to show.
 */
void led_set_status(uint8_t status);
