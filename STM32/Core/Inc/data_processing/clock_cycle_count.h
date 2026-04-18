#pragma once
#include <stdint.h>

/**
 * @brief Resets the internal clock cycle counter and overflow counts
 */
void reset_counter(void);

/**
 * @brief Returns current timestamp in seconds
 * @note Must be called frequently enough to not miss a clock cycle overflow.
 * DWT->CYCCNT overflows every ~25 seconds at 168MHz.
 *
 * @param clock_cycle_count the current clock cycle count of the microcontroller.
 * @return Current timestamp as a double
 */
double process_clock_cycles(uint32_t clock_cycle_count);

/**
 * @brief Tells the clock cycle manager what the current clock speed of the microcontroller is.
 * @note This does not actually set the clock speed of the microcontoller.
 * 
 * @param clock_speed the current clock speed in Mhz.
 */
void use_clock_speed_mhz(uint32_t clock_speed);