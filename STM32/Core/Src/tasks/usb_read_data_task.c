#include "usb_read_data_task.h"

static void firm_system_reset_cb(void *ctx) {
  (void)ctx;
  HAL_NVIC_SystemReset();
}

void usb_read_data(void *argument) {
  uint8_t received_bytes[USB_MESSAGE_BYTE_BUFFER_SIZE];

  // setup system manager commands injection
  commands_register_system_reset(firm_system_reset_cb, NULL);
  
  for (;;) {
    // read the identifier byte to determine payload length
    xStreamBufferReceive(usb_rx_stream, received_bytes, 1, portMAX_DELAY);
    size_t payload_len = parse_message_id(received_bytes[0]);
    if (payload_len == -1) // invalid ID
      continue;

    xStreamBufferReceive(usb_rx_stream, received_bytes + 1, payload_len, portMAX_DELAY);
    uint32_t delay_ms = dispatch_message(received_bytes);
    if (delay_ms > 0U) {
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }
}

void usb_receive_callback(const uint8_t *buffer, uint32_t data_length) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Check if the stream buffer is initialized
  if (usb_rx_stream != NULL) {
    // Adds the received data to the stream buffer from an ISR context
    xStreamBufferSendFromISR(usb_rx_stream, buffer, data_length, &xHigherPriorityTaskWoken);
  }
  // If the usb_read_data task has a higher priority than the currently running task, request a
  // context switch
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
