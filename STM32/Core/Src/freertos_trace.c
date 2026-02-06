#include "freertos_trace.h"
#include "stm32f4xx.h"
#include <string.h>

// Adapted from https://emlogic.no/2025/10/poor-mans-freertos-tracing/

TraceData trace_data;

uint16_t trace_read_timestamp(void) {
  return (uint16_t)(DWT->CYCCNT >> 8);
}

void trace_write_task_switch(int switched_in, const char *task_name) {
  static uint16_t trace_t0 = 0;

  if (switched_in) {
    trace_t0 = trace_read_timestamp();
    return;
  }

  uint32_t index = trace_data.event_index++ % TRACE_EVENT_COUNT;
  TraceEvent *event = &trace_data.events[index];

  memset(event->name, 0, TRACE_TASK_NAME_LEN);
  if (task_name != NULL) {
    // strncpy(event->name, task_name, TRACE_TASK_NAME_LEN - 1U);
    memcpy(event->name, task_name, TRACE_TASK_NAME_LEN); // No null terminator
  }

  event->t0 = trace_t0;
  event->t1 = trace_read_timestamp();
}
