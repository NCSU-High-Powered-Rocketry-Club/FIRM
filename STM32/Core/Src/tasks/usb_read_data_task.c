#include "usb_read_data_task.h"
#include "commands.h"


static bool command_send_to_system_manager(Identifiers_t sys_command) {
  if (sys_manager_check_command_valid(sys_command)) {
    xQueueSend(system_request_queue, &sys_command, portMAX_DELAY);
    return true;
  }
  return false;
}

void usb_read_data(void *argument) {
  uint8_t received_bytes[MESSAGE_BYTE_BUFFER_SIZE];

  // setup system manager commands injection
  commands_set_sys_manager_send_cmd_fn(command_send_to_system_manager);
  
  for (;;) {
    // read the identifier byte to determine payload length
    xStreamBufferReceive(usb_rx_stream, received_bytes, 1, portMAX_DELAY);
    size_t payload_len = parse_message_id(received_bytes[0]);
    if (payload_len == -1)
      continue;

    xStreamBufferReceive(usb_rx_stream, received_bytes + 1, payload_len, portMAX_DELAY);
    if (dispatch_message_get_delay(received_bytes))
      vTaskDelay(pdMS_TO_TICKS(1));
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
