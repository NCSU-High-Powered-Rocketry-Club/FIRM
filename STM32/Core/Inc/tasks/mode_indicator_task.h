#pragma once

#include "task_runtime.h"

extern osThreadId_t firm_mode_indicator_task_handle;
extern const osThreadAttr_t modeIndicatorTask_attributes;

extern QueueHandle_t mode_indicator_command_queue;

void firm_mode_indicator_task(void *argument);
