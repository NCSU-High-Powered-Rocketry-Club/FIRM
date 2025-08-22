/*
 * bmp581_spi.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once
#include "usb_print_debug.h"

int bmp_init();
int bmp_read(SPI_HandleTypeDef *hspi);
