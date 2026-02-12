/*
 * usb_print_debug.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#pragma once

#ifndef TEST
#include "usbd_cdc_if.h"
#endif

#include <string.h>

void serialPrintStr(const char *s);
void serialPrintStrInline(const char *s);
void serialPrintInt(int d);
void serialPrintlnInt(int d);
void serialPrintFloat(float f);
void serialPrintDouble(double d);
void serialPrintlnDouble(double d);
void serialPrintChar(char c);
