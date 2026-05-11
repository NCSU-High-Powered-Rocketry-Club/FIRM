#pragma once

#include "cmsis_os.h"

extern osThreadId_t packetizer_task_handle;
extern const osThreadAttr_t packetizerTask_attributes;

void packetizer_task(void *argument);
