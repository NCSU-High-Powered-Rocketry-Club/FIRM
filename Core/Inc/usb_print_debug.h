/*
 * usb_print_debug.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once
#ifndef INC_USB_PRINT_DEBUG_H_
#define INC_USB_PRINT_DEBUG_H_
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

void serialPrintStr(const char *s);
void serialPrintInt(int d);
void serialPrintlnInt(int d);
void serialPrintFloat(float f);
void serialPrintChar(char c);

#endif /* INC_USB_PRINT_DEBUG_H_ */
