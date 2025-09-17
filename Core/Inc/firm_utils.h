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
 * @brief Turns a 20 bit value into a sign extended 20 bit value
 *
 * @param x unsigned 32 bit integer storing a 20 bit value
 *
 * @retval sign extended 20 bit value
 */
int32_t sign_extend_20bit(uint32_t val);
