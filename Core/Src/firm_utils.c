/*
 * firm_utils.c
 *
 *  Created on: Aug 31, 2025
 *      Author: Wlsan
 */

#include "firm_utils.h"

int16_t twos_complement_16(uint8_t msb, uint8_t lsb) { return (int16_t)((msb << 8) | lsb); }

int32_t sign_extend_20bit(uint32_t val) { return (int32_t)(val << 12) >> 12; }
