/*
 * bmp581_spi.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once

#ifndef INC_BMP581_SPI_H_
#define INC_BMP581_SPI_H_
#include "usb_print_debug.h"

int bmp_init();
int bmp_read(SPI_HandleTypeDef *hspi);

#endif /* INC_BMP581_SPI_H_ */
