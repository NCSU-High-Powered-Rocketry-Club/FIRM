#include "system_state.h"

static volatile SystemState_t g_system_state = SYSTEM_STATE_BOOT;

void system_state_set(SystemState_t state) {
  g_system_state = state;
}

SystemState_t system_state_get(void) {
  return g_system_state;
}
