#pragma once
#include "messages.h"
#include "identifiers.h"
#include "firm_fsm.h"
#include "task_runtime.h"

#define USB_MESSAGE_BYTE_BUFFER_SIZE 128U

extern osThreadId_t usb_read_task_handle;
extern const osThreadAttr_t usbReadTask_attributes;

void usb_read_data(void *argument);
void usb_receive_callback(const uint8_t *buffer, uint32_t data_length);