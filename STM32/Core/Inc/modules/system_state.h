#pragma once

#include <stdint.h>

typedef enum {
  SYSTEM_STATE_BOOT = 0,
  SYSTEM_STATE_LIVE = 1,
  SYSTEM_STATE_MOCK = 2,
} SystemState_t;

void system_state_set(SystemState_t state);
SystemState_t system_state_get(void);
