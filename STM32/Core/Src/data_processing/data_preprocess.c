#include "data_preprocess.h"
#include "settings_manager.h"

// number of times the DWT timestamp has overflowed. This happens every ~25 seconds
static volatile uint32_t dwt_overflow_count = 0;
// last recorded DWT cycle count
static volatile uint32_t last_cyccnt = 0;

/**
 * @brief Updates dwt overflow counter and returns current timestamp
 * @note Called frequently enough that we can't miss an overflow.
 * DWT->CYCCNT overflows every ~25 seconds at 168MHz.
 *
 * @return Current timestamp as a double
 */
static double update_dwt_timestamp(const uint8_t clock_cycle_count[4]) {
  uint32_t current_cyccnt;
  memcpy(&current_cyccnt, clock_cycle_count, sizeof(current_cyccnt));
  // Check for overflow by comparing with last value
  // Overflow occurred if current value is less than last value
  if (current_cyccnt < last_cyccnt) {
    dwt_overflow_count++;
  }
  last_cyccnt = current_cyccnt;
  uint64_t cycle_count = ((uint64_t)dwt_overflow_count << 32) | current_cyccnt;
  // MCU clock speed is 168MHz
  return ((double)cycle_count) / 168000000.0F;
}