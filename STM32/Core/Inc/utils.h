#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Clamps a uint16_t value between min_value and max_value.
 *
 * @param value The input value to clamp.
 * @param min_value The minimum allowable value.
 * @param max_value The maximum allowable value.
 * @return The clamped value.
 */
uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value);

/**
 * Compute CRC-16-CCITT (KERMIT) over the data buffer.
 *
 * @param data Pointer to input data (excluding CRC).
 * @param len Length in bytes.
 * @return 16-bit CRC (transmit LSB first).
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);
