#pragma once
#include "messages.h"
#include "shared_data/identifiers.h"

#include "cmsis_os.h"
#include "stream_buffer.h"

#define USB_RX_STREAM_BUFFER_SIZE_BYTES 6144U
#define USB_RX_STREAM_TRIGGER_LEVEL_BYTES 1U

#define USB_MESSAGE_BYTE_BUFFER_SIZE 128U

extern osThreadId_t usb_read_task_handle;
extern const osThreadAttr_t usbReadTask_attributes;

extern StreamBufferHandle_t usb_rx_stream;

void usb_read_data(void *argument);
void usb_receive_callback(const uint8_t *buffer, uint32_t data_length);