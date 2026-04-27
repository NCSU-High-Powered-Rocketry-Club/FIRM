#pragma once

#include "task_runtime.h"

extern osThreadId_t transmit_task_handle;
extern const osThreadAttr_t transmitTask_attributes;

void transmit_data(void *argument);
