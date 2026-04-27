#pragma once

#include "task_runtime.h"

extern osThreadId_t mock_packet_handler_handle;
extern const osThreadAttr_t mockPacketTask_attributes;

extern QueueHandle_t mock_packet_handler_command_queue;

void mock_packet_handler(void *argument);
