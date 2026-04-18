#include "clock_cycle_count.h"

// number of times the DWT timestamp has overflowed. This happens every ~25 seconds
static volatile uint32_t dwt_overflow_count = 0;
// last recorded DWT cycle count
static volatile uint32_t last_cyccnt = 0;

static uint32_t clock_speed_hz = 168000000;

double process_clock_cycles(uint32_t clock_cycle_count) {
  // Check for overflow by comparing with last value
  // Overflow occurred if current value is less than last value
  if (clock_cycle_count < last_cyccnt) {
    dwt_overflow_count++;
  }
  last_cyccnt = clock_cycle_count;

  // using bit concatenation to combine the number of overflows (upper 32 bits) and the clock
  // cycle count (lower 32 bits) to make a 64 bit number.
  uint64_t cycle_count = ((uint64_t)dwt_overflow_count << 32) | clock_cycle_count;
  
  // divide by clock speed to convert to seconds
  return ((double)cycle_count) / clock_speed_hz;
}

void use_clock_speed_mhz(uint32_t clock_speed) {
  clock_speed_hz = clock_speed * 1000000;
}