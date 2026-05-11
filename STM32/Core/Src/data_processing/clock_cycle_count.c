#include "clock_cycle_count.h"
#include <stddef.h>

void clock_cycle_counter_init(ClockCycleCounter_t *counter, uint32_t clock_speed_mhz) {
  counter->dwt_overflow_count = 0U;
  counter->last_cyccnt = 0U;
  counter->clock_speed_hz = clock_speed_mhz * 1000000U;
}

void clock_cycle_counter_reset(ClockCycleCounter_t *counter) {
  counter->dwt_overflow_count = 0U;
  counter->last_cyccnt = 0U;
}

void clock_cycle_counter_set_speed_mhz(ClockCycleCounter_t *counter, uint32_t clock_speed_mhz) {
  counter->clock_speed_hz = clock_speed_mhz * 1000000U;
}

double clock_cycle_counter_process(ClockCycleCounter_t *counter, uint32_t clock_cycle_count) {
  // Check for overflow by comparing with last value
  // Overflow occurred if current value is less than last value
  if (clock_cycle_count < counter->last_cyccnt) {
    counter->dwt_overflow_count++;
  }
  counter->last_cyccnt = clock_cycle_count;

  // using bit concatenation to combine the number of overflows (upper 32 bits) and the clock
  // cycle count (lower 32 bits) to make a 64 bit number.
  uint64_t cycle_count = ((uint64_t)counter->dwt_overflow_count << 32) | clock_cycle_count;
  
  // divide by clock speed to convert to seconds
  return ((double)cycle_count) / counter->clock_speed_hz;
}