#pragma once

#include "cmsis_os.h"

extern osThreadId_t firm_mode_indicator_task_handle;
extern const osThreadAttr_t modeIndicatorTask_attributes;

void firm_mode_indicator_task(void *argument);
