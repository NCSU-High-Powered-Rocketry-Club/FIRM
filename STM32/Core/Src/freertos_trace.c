#include "freertos_trace.h"
#include "stm32f4xx.h"
#include <string.h>

// Adapted from https://emlogic.no/2025/10/poor-mans-freertos-tracing/

TraceData trace_data;

uint16_t region_stack[16] = {0};
int region_stack_count = 0;

uint16_t trace_read_timestamp(void) { return (uint16_t)(DWT->CYCCNT >> 8); }

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

// TODO: this is only going to work single threaded
void trace_begin_region() {
  // Overflowing, ignore
  if (region_stack_count >= sizeof(region_stack) / sizeof(uint16_t)) {
    return;
  }

  region_stack[region_stack_count] = trace_read_timestamp();
  region_stack_count++;
}

void trace_end_region(const char id[2]) {
  if (region_stack_count == 0) {
    // Underflowing, ignore
    return;
  }

  uint16_t end_time = trace_read_timestamp();
  uint16_t start_time = region_stack[region_stack_count - 1];

  uint32_t index = trace_data.event_index++ % TRACE_EVENT_COUNT;
  TraceEvent *event = &trace_data.events[index];
  event->name[0] = id[0];
  event->name[0] |= 1 << 7; // Set highest bit to 1 to differentiate from tasks
  event->name[1] = id[1];
  event->t0 = start_time;
  event->t1 = end_time;

  region_stack_count--;
}