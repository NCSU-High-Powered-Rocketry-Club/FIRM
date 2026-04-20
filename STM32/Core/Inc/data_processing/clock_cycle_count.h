#pragma once
#include <stdint.h>

/**
 * @brief Instance state for clock-cycle to seconds conversion.
 */
typedef struct {
	uint32_t dwt_overflow_count;
	uint32_t last_cyccnt;
	uint32_t clock_speed_hz;
} ClockCycleCounter_t;

/**
 * @brief Initializes a clock cycle counter instance.
 *
 * @param counter Pointer to counter state.
 * @param clock_speed_mhz CPU clock speed in MHz.
 */
void clock_cycle_counter_init(ClockCycleCounter_t *counter, uint32_t clock_speed_mhz);

/**
 * @brief Resets overflow and last-cycle tracking for an instance.
 *
 * @param counter Pointer to counter state.
 */
void clock_cycle_counter_reset(ClockCycleCounter_t *counter);

/**
 * @brief Updates clock speed used by an instance.
 * @note This does not set MCU hardware speed; it updates conversion math only.
 *
 * @param counter Pointer to counter state.
 * @param clock_speed_mhz CPU clock speed in MHz.
 */
void clock_cycle_counter_set_speed_mhz(ClockCycleCounter_t *counter, uint32_t clock_speed_mhz);

/**
 * @brief Returns timestamp in seconds for this counter instance.
 * @note Must be called frequently enough to not miss a clock cycle overflow.
 * DWT->CYCCNT overflows every ~25 seconds at 168MHz.
 *
 * @param counter Pointer to counter state.
 * @param clock_cycle_count Current DWT cycle count.
 * @return Current timestamp as a double
 */
double clock_cycle_counter_process(ClockCycleCounter_t *counter, uint32_t clock_cycle_count);