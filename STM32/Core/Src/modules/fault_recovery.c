#include "fault_recovery.h"

#include <stddef.h>

#define FAULT_RECORD_MAGIC 0xFA11BAC4U

// The checksum covers every byte before it, so there must be no padding in between.
_Static_assert(offsetof(FaultRecord_t, check) == sizeof(FaultRecord_t) - sizeof(uint32_t),
               "FaultRecord_t must not have padding before check");

// 32-bit FNV-1a over everything before the check field.
static uint32_t record_check(const FaultRecord_t *record) {
  const uint8_t *bytes = (const uint8_t *)record;
  uint32_t hash = 2166136261U;
  for (size_t i = 0; i < offsetof(FaultRecord_t, check); i++) {
    hash ^= bytes[i];
    hash *= 16777619U;
  }
  return hash;
}

static bool record_valid(const FaultRecord_t *record) {
  return record->magic == FAULT_RECORD_MAGIC && record->check == record_check(record);
}

static void copy_task_name(char *dest, const char *name) {
  size_t i = 0;
  if (name != NULL) {
    for (; i < FAULT_TASK_NAME_LEN - 1U && name[i] != '\0'; i++) {
      dest[i] = name[i];
    }
  }
  for (; i < FAULT_TASK_NAME_LEN; i++) {
    dest[i] = '\0';
  }
}

FaultAction_t fault_recovery_on_stack_overflow(FaultRecord_t *record, const char *task_name,
                                               uint32_t uptime_ms, bool debugger_attached) {
  if (record == NULL || debugger_attached) {
    return FAULT_ACTION_HALT;
  }

  uint32_t consecutive = 1U;
  if (record_valid(record) && uptime_ms < FAULT_BOOT_LOOP_WINDOW_MS) {
    consecutive = (record->consecutive < UINT32_MAX) ? record->consecutive + 1U : UINT32_MAX;
  }

  record->magic = FAULT_RECORD_MAGIC;
  record->consecutive = consecutive;
  record->uptime_ms = uptime_ms;
  copy_task_name(record->task_name, task_name);
  record->check = record_check(record);

  return (consecutive > FAULT_MAX_CONSECUTIVE_RESETS) ? FAULT_ACTION_HALT : FAULT_ACTION_RESET;
}
