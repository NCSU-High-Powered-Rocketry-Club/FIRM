#pragma once

#include <stdint.h>

// LED status definitions
// 3-bit status codes for the 3 LEDs Blue, Yellow, and Red - in order.
#define UNINITIALIZED   0b111
#define IMU_FAIL        0b110
#define BMP581_FAIL     0b101
#define MMC5983MA_FAIL  0b100
#define FLASH_CHIP_FAIL 0b011
#define SD_CARD_FAIL    0b010
#define ALL_SENSORS_OK  0b000

/**
 * @brief Sets all 3 LEDs based on the given status byte.
 *
 * @param status 3-bit status code representing the state of the LEDs to show.
 */
void led_set_status(uint8_t status);
