/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include "i2c_utils.h"
#include "usb_print_debug.h"
#include <stdint.h>

int mag_init(I2C_HandleTypeDef *hi2c);

int mag_read(I2C_HandleTypeDef *hi2c, int flip);
