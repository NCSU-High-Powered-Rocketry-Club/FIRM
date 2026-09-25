#include <unity.h>

#include <stddef.h>

#include "clock_cycle_count.h"

static ClockCycleCounter_t counter;

void setUp(void) { clock_cycle_counter_init(&counter, 168); }

void tearDown(void) {}

void test_counter_reset(void) {
  clock_cycle_counter_set_speed_mhz(&counter, 168);
  clock_cycle_counter_process(&counter, 100000000);
  clock_cycle_counter_process(&counter, 50000000); // triggers overflow

  clock_cycle_counter_reset(&counter);
  // use a cycle count that would cause an overflow if the reset did NOT happen (cyccnt2 < cyccnt1)
  double timestamp2 = clock_cycle_counter_process(&counter, 50000000);
  double exp2 = (double)50000000 / (double)168000000;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, timestamp2);
}

void test_no_overflows_single(void) {
  const struct {
    uint32_t cyccnt;
    uint32_t clock_speed_hz;
  } cases[] = {
      {15538932, 168000000},
      {400000, 128000000},
  };

  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    clock_cycle_counter_init(&counter, 168);
    clock_cycle_counter_set_speed_mhz(&counter, cases[i].clock_speed_hz / 1000000U);
    double timestamp = clock_cycle_counter_process(&counter, cases[i].cyccnt);
    double exp = (double)cases[i].cyccnt / (double)cases[i].clock_speed_hz;
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
  }
}

void test_no_overflow_multiple(void) {
  const uint32_t speeds_hz[] = {150000000, 480000000};
  uint32_t cyccnt[5] = {128989, 479385, 3984558, 5389322, 34829571};

  for (size_t s = 0; s < sizeof(speeds_hz) / sizeof(speeds_hz[0]); s++) {
    clock_cycle_counter_init(&counter, 168);
    clock_cycle_counter_set_speed_mhz(&counter, speeds_hz[s] / 1000000U);
    for (int i = 0; i < 5; i++) {
      double timestamp = clock_cycle_counter_process(&counter, cyccnt[i]);
      double exp = (double)cyccnt[i] / (double)speeds_hz[s];
      TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp, timestamp);
    }
  }
}

void test_overflows(void) {
  const uint32_t speeds_hz[] = {180000000, 360000000};
  uint32_t cyccnt[5] = {50000000, 100000000, 20000000, 60000000, 30000000};

  for (size_t s = 0; s < sizeof(speeds_hz) / sizeof(speeds_hz[0]); s++) {
    const uint32_t hz = speeds_hz[s];
    clock_cycle_counter_init(&counter, 168);
    clock_cycle_counter_set_speed_mhz(&counter, hz / 1000000U);
    const double sec_per_overflow = (double)4294967296 / (double)hz;

    double t0 = clock_cycle_counter_process(&counter, cyccnt[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[0] / (double)hz, t0);
    double t1 = clock_cycle_counter_process(&counter, cyccnt[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, (double)cyccnt[1] / (double)hz, t1);

    double exp2 = (double)cyccnt[2] / (double)hz + sec_per_overflow;
    double t2 = clock_cycle_counter_process(&counter, cyccnt[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp2, t2);
    double exp3 = (double)cyccnt[3] / (double)hz + sec_per_overflow;
    double t3 = clock_cycle_counter_process(&counter, cyccnt[3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp3, t3);

    double exp4 = (double)cyccnt[4] / (double)hz + sec_per_overflow * 2;
    double t4 = clock_cycle_counter_process(&counter, cyccnt[4]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, exp4, t4);
  }
}
