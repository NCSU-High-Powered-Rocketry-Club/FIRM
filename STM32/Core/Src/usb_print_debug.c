/*
 * usb_print_debug.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "usb_print_debug.h"

void serialPrintStr(const char* s) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%s\r\n", s);
    // debug messages dont work well if they are right after another, so this makes sure
    // the message is printed, and tries again if the attempt failed
    int retry_count = 0;
    int ret = 1;
    const int MAX_RETRIES = 100;
    while (ret && retry_count < MAX_RETRIES) {
        ret = CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        retry_count++;
    }
}


void serialPrintInt(int d, bool newLine) {
    char buffer[32];

    if (newLine == false) {
        sprintf(buffer, "%d", d);
    }
    else {
        snprintf(buffer, sizeof(buffer), "%d\r\n", d);
    }
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}


void serialPrintFloat(float f) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%f\r\n", f);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

void serialPrintChar(char c) {
    char buffer[4];
    snprintf(buffer, sizeof(buffer), "%c\r\n", c);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}
