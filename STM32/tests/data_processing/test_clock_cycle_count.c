#include "clock_cycle_count.h"
#include "utest.h"

struct clock_cycle_count {
  ClockCycleCounter_t counter;
  size_t case_index;
};

UTEST_F_SETUP(clock_cycle_count) { clock_cycle_counter_init(&utest_fixture->counter, 168); }
UTEST_F_TEARDOWN(clock_cycle_count) {}

UTEST_I_SETUP(clock_cycle_count) {
  clock_cycle_counter_init(&utest_fixture->counter, 168);
  utest_fixture->case_index = utest_index;
}
UTEST_I_TEARDOWN(clock_cycle_count) {}

UTEST_F(clock_cycle_count, counter_reset) {
  ClockCycleCounter_t *counter = &utest_fixture->counter;
  clock_cycle_counter_set_speed_mhz(counter, 168);
  clock_cycle_counter_process(counter, 100000000);
  clock_cycle_counter_process(counter, 50000000); // triggers overflow

  // reset the counter
  clock_cycle_counter_reset(counter);
  // use a cycle count that would cause an overflow if the reset did NOT happen (cyccnt2 < cyccnt1)
  double timestamp2 = clock_cycle_counter_process(counter, 50000000);
  double exp2 = (double)50000000 / (double)168000000;
  ASSERT_NEAR(exp2, timestamp2, 1e-6);
}

static const struct {
  uint32_t cyccnt;
  uint32_t clock_speed_hz;
} no_overflows_single_cases[] = {{15538932, 168000000}, {400000, 128000000}};

UTEST_I(clock_cycle_count, no_overflows_single, 2) {
  uint32_t cyccnt = no_overflows_single_cases[utest_fixture->case_index].cyccnt;
  uint32_t clock_speed_hz = no_overflows_single_cases[utest_fixture->case_index].clock_speed_hz;
  ClockCycleCounter_t *counter = &utest_fixture->counter;

  // clock speed input is in MHz, divide by 1e6
  clock_cycle_counter_set_speed_mhz(counter, clock_speed_hz / 1000000U);
  double timestamp = clock_cycle_counter_process(counter, cyccnt);
  double exp = (double)cyccnt / (double)clock_speed_hz;
  ASSERT_NEAR(exp, timestamp, 1e-6);
}

static const uint32_t no_overflow_multiple_cases[] = {150000000, 480000000};

UTEST_I(clock_cycle_count, no_overflow_multiple, 2) {
  uint32_t clock_speed_hz = no_overflow_multiple_cases[utest_fixture->case_index];
  ClockCycleCounter_t *counter = &utest_fixture->counter;

  clock_cycle_counter_set_speed_mhz(counter, clock_speed_hz / 1000000U);
  uint32_t cyccnt[5] = {128989, 479385, 3984558, 5389322, 34829571};

  for (int i = 0; i < 5; i++) {
    double timestamp = clock_cycle_counter_process(counter, cyccnt[i]);
    double exp = (double)cyccnt[i] / (double)clock_speed_hz;
    ASSERT_NEAR(exp, timestamp, 1e-6);
  }
}

static const uint32_t overflows_cases[] = {180000000, 360000000};

UTEST_I(clock_cycle_count, overflows, 2) {
  uint32_t hz = overflows_cases[utest_fixture->case_index];
  ClockCycleCounter_t *counter = &utest_fixture->counter;

  clock_cycle_counter_set_speed_mhz(counter, hz / 1000000U);
  // 50M -> 100M -> overflow to 20M -> 60M -> overflow to 30M
  uint32_t cyccnt[5] = {50000000, 100000000, 20000000, 60000000, 30000000};
  const double sec_per_overflow = (double)4294967296 / (double)hz;

  double t0 = clock_cycle_counter_process(counter, cyccnt[0]);
  ASSERT_NEAR((double)cyccnt[0] / (double)hz, t0, 1e-6);
  double t1 = clock_cycle_counter_process(counter, cyccnt[1]);
  ASSERT_NEAR((double)cyccnt[1] / (double)hz, t1, 1e-6);

  // overflows
  double exp2 = (double)cyccnt[2] / (double)hz + sec_per_overflow;
  double t2 = clock_cycle_counter_process(counter, cyccnt[2]);
  ASSERT_NEAR(exp2, t2, 1e-6);
  double exp3 = (double)cyccnt[3] / (double)hz + sec_per_overflow;
  double t3 = clock_cycle_counter_process(counter, cyccnt[3]);
  ASSERT_NEAR(exp3, t3, 1e-6);

  // overflows again
  double exp4 = (double)cyccnt[4] / (double)hz + sec_per_overflow * 2;
  double t4 = clock_cycle_counter_process(counter, cyccnt[4]);
  ASSERT_NEAR(exp4, t4, 1e-6);
}

UTEST_MAIN()
