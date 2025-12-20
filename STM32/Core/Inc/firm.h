#pragma once

#include <stdbool.h>
#include "stm32f4xx_hal.h"




/**
 * @brief The main loop which checks if any of the sensors have new data, reads it, logs it, and
 *        sends it over USB if connected.
 */
void loop_firm(void);