#pragma once

#include "usb_print_debug.h"

typedef enum {
    STANDBY,
    MOTOR_BURN,
    COAST,
    FREE_FALL,
    LANDED,
    NUM_STATES
} State;

void update_state(void);
