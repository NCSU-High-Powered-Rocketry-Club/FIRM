#include <unity.h>
#include "clock_cycle_count.h"

static ClockCycleCounter_t counter;

void setUp() {
  clock_cycle_counter_init(&counter, 168);
}
void tearDown() {}


void test_counter_reset() {
  clock_cycle_counter_set_speed_mhz(&counter, 168);
  clock_cycle_counter_process(&counter, 100000000);
  clock_cycle_counter_process(&counter, 50000000); // triggers overflow

  // reset the counter
  clock_cycle_counter_reset(&counter);
  // use a cycle count that would cause an overflow if the reset did NOT happen (cyccnt2 < cyccnt1)
  double timestamp2 = clock_cycle_counter_process(&counter, 50000000);
  double exp2 = (double)50000000 / (double)168000000;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, timestamp2);
}

TEST_CASE(15538932, 168000000)
TEST_CASE(400000, 128000000)
void test_no_overflows_single(uint32_t cyccnt, uint32_t clock_speed_hz) {
  // clock speed input is in MHz, divide by 1e6
  clock_cycle_counter_set_speed_mhz(&counter, clock_speed_hz / 1000000U);
  double timestamp = clock_cycle_counter_process(&counter, cyccnt);
  double exp = (double)cyccnt / (double)clock_speed_hz;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
}

TEST_CASE(150000000)
TEST_CASE(480000000)
void test_no_overflow_multiple(uint32_t clock_speed_hz) {
  clock_cycle_counter_set_speed_mhz(&counter, clock_speed_hz / 1000000U);
  uint32_t cyccnt[5] = {128989, 479385, 3984558, 5389322, 34829571};

  for (int i = 0; i < 5; i++) {
    double timestamp = clock_cycle_counter_process(&counter, cyccnt[i]);
    double exp = (double)cyccnt[i] / (double)clock_speed_hz;
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
  }
}

TEST_CASE(180000000)
TEST_CASE(360000000)
void test_overflows(uint32_t hz) {
  clock_cycle_counter_set_speed_mhz(&counter, hz / 1000000U);
  // 50M -> 100M -> overflow to 20M -> 60M -> overflow to 30M
  uint32_t cyccnt[5] = {50000000, 100000000, 20000000, 60000000, 30000000};
  const double sec_per_overflow = (double)4294967296 / (double)hz;

  double t0 = clock_cycle_counter_process(&counter, cyccnt[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[0] / (double)hz, t0);
  double t1 = clock_cycle_counter_process(&counter, cyccnt[1]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[1] / (double)hz, t1);

  // overflows
  double exp2 = (double)cyccnt[2] / (double)hz + sec_per_overflow;
  double t2 = clock_cycle_counter_process(&counter, cyccnt[2]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, t2);
  double exp3 = (double)cyccnt[3] / (double)hz + sec_per_overflow;
  double t3 = clock_cycle_counter_process(&counter, cyccnt[3]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp3, t3);

  // overflows again
  double exp4 = (double)cyccnt[4] / (double)hz + sec_per_overflow * 2;
  double t4 = clock_cycle_counter_process(&counter, cyccnt[4]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp4, t4);
}