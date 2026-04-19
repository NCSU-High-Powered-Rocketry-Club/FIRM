#include <unity.h>
#include "clock_cycle_count.h"

void setUp() {
  reset_counter();
}
void tearDown() {}


void test_counter_reset() {
  use_clock_speed_mhz(168);
  process_clock_cycles(100000000);
  process_clock_cycles(50000000); // triggers overflow

  // reset the counter
  reset_counter();
  // use a cycle count that would cause an overflow if the reset did NOT happen (cyccnt2 < cyccnt1)
  double timestamp2 = process_clock_cycles(50000000);
  double exp2 = (double)50000000 / (double)168000000;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, timestamp2);
}

TEST_CASE(15538932, 168000000)
TEST_CASE(400000, 128000000)
void test_no_overflows_single(uint32_t cyccnt, uint32_t clock_speed_hz) {
  // clock speed input is in MHz, divide by 1e6
  use_clock_speed_mhz(clock_speed_hz / 1000000U);
  double timestamp = process_clock_cycles(cyccnt);
  double exp = (double)cyccnt / (double)clock_speed_hz;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
}

TEST_CASE(150000000)
TEST_CASE(480000000)
void test_no_overflow_multiple(uint32_t clock_speed_hz) {
  use_clock_speed_mhz(clock_speed_hz / 1000000U);
  uint32_t cyccnt[5] = {128989, 479385, 3984558, 5389322, 34829571};

  for (int i = 0; i < 5; i++) {
    double timestamp = process_clock_cycles(cyccnt[i]);
    double exp = (double)cyccnt[i] / (double)clock_speed_hz;
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
  }
}

TEST_CASE(180000000)
TEST_CASE(360000000)
void test_overflows(uint32_t hz) {
  use_clock_speed_mhz(hz / 1000000U);
  // 50M -> 100M -> overflow to 20M -> 60M -> overflow to 30M
  uint32_t cyccnt[5] = {50000000, 100000000, 20000000, 60000000, 30000000};
  const double sec_per_overflow = (double)4294967296 / (double)hz;

  double t0 = process_clock_cycles(cyccnt[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[0] / (double)hz, t0);
  double t1 = process_clock_cycles(cyccnt[1]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[1] / (double)hz, t1);

  // overflows
  double exp2 = (double)cyccnt[2] / (double)hz + sec_per_overflow;
  double t2 = process_clock_cycles(cyccnt[2]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, t2);
  double exp3 = (double)cyccnt[3] / (double)hz + sec_per_overflow;
  double t3 = process_clock_cycles(cyccnt[3]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp3, t3);

  // overflows again
  double exp4 = (double)cyccnt[4] / (double)hz + sec_per_overflow * 2;
  double t4 = process_clock_cycles(cyccnt[4]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp4, t4);
}