#pragma once

#include <stdint.h>

#define TRACE_TASK_NAME_LEN 2U
// #define TRACE_EVENT_COUNT 1024U
#define TRACE_EVENT_COUNT 800U

typedef struct {
  char name[TRACE_TASK_NAME_LEN];
  uint16_t t0;
  uint16_t t1;
} TraceEvent;

typedef struct {
  TraceEvent events[TRACE_EVENT_COUNT];
  uint32_t event_index;
} TraceData;

extern TraceData trace_data;

void trace_write_task_switch(int switched_in, const char *task_name);
uint16_t trace_read_timestamp(void);
