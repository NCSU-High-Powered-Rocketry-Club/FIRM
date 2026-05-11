#pragma once

#include "cmsis_os.h"
#include "queue.h"

#define TRANSMIT_QUEUE_LENGTH 10U

extern osThreadId_t transmit_task_handle;
extern const osThreadAttr_t transmitTask_attributes;

extern QueueHandle_t transmit_queue;

void transmit_data(void *argument);
