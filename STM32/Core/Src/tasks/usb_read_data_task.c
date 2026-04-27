#include "usb_read_data_task.h"
#include "system_manager_task.h"

static bool identifier_to_system_request(Identifiers_t id, SystemRequest *request_out) {
  if (request_out == NULL) {
    return false;
  }

  switch (id) {
  case ID_MOCK_REQUEST:
    *request_out = SYSREQ_START_MOCK;
    return true;
  case ID_CANCEL_REQUEST:
    *request_out = SYSREQ_CANCEL;
    return true;
  default:
    return false;
  }
}

static bool command_send_to_system_manager(Identifiers_t sys_command) {
  SystemRequest request;
  if (!identifier_to_system_request(sys_command, &request)) {
    return false;
  }

  return system_manager_submit_request(request, portMAX_DELAY);
}

void usb_read_data(void *argument) {
  uint8_t received_bytes[USB_MESSAGE_BYTE_BUFFER_SIZE];

  // setup system manager commands injection
  commands_set_sys_manager_send_cmd_fn(command_send_to_system_manager);
  
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
