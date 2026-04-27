#include "transmit_frame.h"
#include "commands.h"
#include "transmit_task.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

static void transmit_send_to_queue(TransmitFrame_t *transmit_frame) {
  xQueueSend(transmit_queue, transmit_frame, portMAX_DELAY);
}

void transmit_data(void *arg) {
  TransmitFrame_t transmit_frame;


  // when task starts, inject the queue sending function to the appropriate areas
  commands_set_response_queue(transmit_send_to_queue);
  for (;;) {
    // task wakes up when new transmit frame in the queue
    if (xQueueReceive(transmit_queue, &transmit_frame, portMAX_DELAY) == pdTRUE) {

      // If the USB is busy, we might need to try again in a tick
      for (int timeout = 0; timeout < 5; timeout++) {
        if (CDC_Transmit_FS(transmit_frame.payload, transmit_frame.payload_len) == USBD_OK)
          break;
        vTaskDelay(1);
      }
    }
  }
}