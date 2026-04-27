#pragma once

#include "task_runtime.h"

extern osThreadId_t system_manager_task_handle;
extern const osThreadAttr_t systemManagerTask_attributes;

bool system_manager_submit_request(SystemRequest request, TickType_t timeout_ticks);
void system_manager_task(void *argument);
