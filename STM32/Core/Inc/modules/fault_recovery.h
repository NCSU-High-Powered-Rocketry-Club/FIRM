#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FAULT_TASK_NAME_LEN 16U

// A fault this soon after boot counts as part of a reset loop.
#define FAULT_BOOT_LOOP_WINDOW_MS 30000U

// Resets allowed in a row before giving up and halting.
#define FAULT_MAX_CONSECUTIVE_RESETS 3U

typedef enum {
  FAULT_ACTION_RESET,
  FAULT_ACTION_HALT,
} FaultAction_t;

// Lives in RAM that survives a software reset (.noinit), so it carries its own validity check:
// after a power-up its contents are random.
typedef struct {
  uint32_t magic;
  uint32_t consecutive; // faults in a row that each happened soon after boot
  uint32_t uptime_ms;   // uptime when the last fault hit
  char task_name[FAULT_TASK_NAME_LEN];
  uint32_t check;
} FaultRecord_t;

/**
 * @brief Decides whether to reset or halt after a stack overflow, and updates the record.
 *
 * With a debugger attached the answer is always HALT and the record is left alone, so a debug
 * session doesn't use up the reset budget. Otherwise the answer is RESET until the overflows keep
 * recurring within FAULT_BOOT_LOOP_WINDOW_MS of boot more than FAULT_MAX_CONSECUTIVE_RESETS times
 * in a row, then HALT. An overflow after a longer healthy run starts the count over.
 *
 * @param record Record in reset-surviving RAM. NULL means HALT.
 * @param task_name Name of the overflowing task. May be NULL.
 * @param uptime_ms Milliseconds since boot.
 * @param debugger_attached True when a debugger is connected.
 */
FaultAction_t fault_recovery_on_stack_overflow(FaultRecord_t *record, const char *task_name,
                                               uint32_t uptime_ms, bool debugger_attached);
