#pragma once

#include <stdint.h>

#define TRACE_TASK_NAME_LEN 2U
// #define TRACE_EVENT_COUNT 1024U
#define TRACE_EVENT_COUNT 800U

void trace_begin_region();
void trace_end_region(const char id[2]);

#define TRACE_BEGIN_REGION() trace_begin_region();
#define TRACE_END_REGION(id, name) trace_end_region(id);

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