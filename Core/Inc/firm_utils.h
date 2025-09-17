/*
 * firm_utils.h
 *
 *  Created on: Aug 31, 2025
 *      Author: Wlsan
 */

#pragma once
#include <stdint.h>

/**
 * @brief Combine two bytes into a signed 16-bit integer using two's complement.
 *
 * @param msb the most significant byte of the 16-bit value.
 * @param lsb the least signifcant byte of the 16-bit value.
 *
 * @retval the signed 16-bit integer.
 */
int16_t twos_complement_16(uint8_t msb, uint8_t lsb);

/**
 * @brief Converts an unsigned 20-bit value to a signed 32-bit integer by sign-extending the 19th
 * bit.
 *
 * This function takes an unsigned 32-bit integer containing a 20-bit value (bits 0–19)
 * and returns a signed 32-bit integer, where the sign bit (bit 19) is extended to bits 20–31.
 *
 * @param val Unsigned 32-bit integer storing a 20-bit value (bits 0–19).
 *
 * @retval Signed 32-bit integer with the value of the input, sign-extended from bit 19.
 */
int32_t sign_extend_20bit(uint32_t val);
