#include <string.h>

#include "fault_recovery.h"
#include "utest.h"

// Power-up RAM is random, so tests start from a deliberately garbage record.
static void make_garbage(FaultRecord_t *record) { memset(record, 0xA5, sizeof(*record)); }

UTEST(fault_recovery, first_overflow_after_power_up_resets_and_records_the_task) {
  FaultRecord_t record;
  make_garbage(&record);

  ASSERT_EQ(FAULT_ACTION_RESET,
            fault_recovery_on_stack_overflow(&record, "sensorTask", 5000U, false));
  ASSERT_EQ(1U, record.consecutive);
  ASSERT_EQ(5000U, record.uptime_ms);
  ASSERT_STREQ("sensorTask", record.task_name);
}

UTEST(fault_recovery, quick_repeats_reset_until_the_budget_is_spent) {
  FaultRecord_t record;
  make_garbage(&record);

  for (uint32_t i = 1U; i <= FAULT_MAX_CONSECUTIVE_RESETS; i++) {
    ASSERT_EQ(FAULT_ACTION_RESET,
              fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false));
    ASSERT_EQ(i, record.consecutive);
  }

  ASSERT_EQ(FAULT_ACTION_HALT,
            fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false));
  ASSERT_EQ(FAULT_MAX_CONSECUTIVE_RESETS + 1U, record.consecutive);
}

UTEST(fault_recovery, a_halted_loop_stays_halted_on_further_quick_overflows) {
  FaultRecord_t record;
  make_garbage(&record);
  for (uint32_t i = 0U; i <= FAULT_MAX_CONSECUTIVE_RESETS; i++) {
    (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);
  }

  ASSERT_EQ(FAULT_ACTION_HALT,
            fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false));
}

UTEST(fault_recovery, overflow_after_a_long_healthy_run_starts_the_count_over) {
  FaultRecord_t record;
  make_garbage(&record);
  for (uint32_t i = 0U; i < FAULT_MAX_CONSECUTIVE_RESETS; i++) {
    (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);
  }
  ASSERT_EQ(FAULT_MAX_CONSECUTIVE_RESETS, record.consecutive);

  ASSERT_EQ(FAULT_ACTION_RESET, fault_recovery_on_stack_overflow(&record, "sensorTask",
                                                                 FAULT_BOOT_LOOP_WINDOW_MS, false));
  ASSERT_EQ(1U, record.consecutive);
}

UTEST(fault_recovery, window_boundary_counts_just_inside_as_a_loop) {
  FaultRecord_t record;
  make_garbage(&record);
  (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);

  ASSERT_EQ(FAULT_ACTION_RESET, fault_recovery_on_stack_overflow(
                                    &record, "sensorTask", FAULT_BOOT_LOOP_WINDOW_MS - 1U, false));
  ASSERT_EQ(2U, record.consecutive);
}

UTEST(fault_recovery, debugger_halts_and_leaves_the_record_untouched) {
  FaultRecord_t record;
  make_garbage(&record);
  (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);
  (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);
  FaultRecord_t before = record;

  ASSERT_EQ(FAULT_ACTION_HALT, fault_recovery_on_stack_overflow(&record, "other", 2000U, true));
  ASSERT_EQ(0, memcmp(&before, &record, sizeof(record)));
}

UTEST(fault_recovery, debugger_halts_even_on_a_garbage_record) {
  FaultRecord_t record;
  make_garbage(&record);

  ASSERT_EQ(FAULT_ACTION_HALT, fault_recovery_on_stack_overflow(&record, "x", 100U, true));
}

UTEST(fault_recovery, tampered_count_is_not_trusted) {
  FaultRecord_t record;
  make_garbage(&record);
  (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);

  record.consecutive = FAULT_MAX_CONSECUTIVE_RESETS; // changed without updating the checksum

  ASSERT_EQ(FAULT_ACTION_RESET,
            fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false));
  ASSERT_EQ(1U, record.consecutive);
}

UTEST(fault_recovery, wrong_magic_is_not_trusted) {
  FaultRecord_t record;
  make_garbage(&record);
  (void)fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false);

  record.magic ^= 1U;

  ASSERT_EQ(FAULT_ACTION_RESET,
            fault_recovery_on_stack_overflow(&record, "sensorTask", 1000U, false));
  ASSERT_EQ(1U, record.consecutive);
}

UTEST(fault_recovery, long_task_name_is_truncated_and_terminated) {
  FaultRecord_t record;
  make_garbage(&record);

  (void)fault_recovery_on_stack_overflow(&record, "abcdefghijklmnopqrstuvwxyz", 1000U, false);

  ASSERT_STREQ("abcdefghijklmno", record.task_name);
  ASSERT_EQ('\0', record.task_name[FAULT_TASK_NAME_LEN - 1U]);
}

UTEST(fault_recovery, missing_task_name_is_stored_as_empty) {
  FaultRecord_t record;
  make_garbage(&record);

  ASSERT_EQ(FAULT_ACTION_RESET, fault_recovery_on_stack_overflow(&record, NULL, 1000U, false));
  ASSERT_STREQ("", record.task_name);
}

UTEST(fault_recovery, missing_record_halts) {
  ASSERT_EQ(FAULT_ACTION_HALT, fault_recovery_on_stack_overflow(NULL, "x", 1000U, false));
}

UTEST_MAIN()
