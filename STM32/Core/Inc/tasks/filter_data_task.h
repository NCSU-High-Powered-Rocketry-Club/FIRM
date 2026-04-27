#pragma once

#include "task_runtime.h"

#define KALMAN_FILTER_STARTUP_DELAY_TIME_MS 2000

extern osThreadId_t filter_data_task_handle;
extern const osThreadAttr_t filterDataTask_attributes;

extern QueueHandle_t data_filter_command_queue;

void filter_data_task(void *argument);
