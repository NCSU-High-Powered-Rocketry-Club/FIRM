#include "packetizer_task.h"

#include "messages.h"
#include "settings_manager.h"
#include "transmit_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000U / (hz)) + 1U)

osThreadId_t packetizer_task_handle;
const osThreadAttr_t packetizerTask_attributes = {
  .name = "packetizerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityHigh,
};

void packetizer_task(void *argument) {

  const SystemSettings_t *settings = get_settings();
  uint16_t frequency_hz = (settings->frequency_hz == 0U) ? 1U : settings->frequency_hz;
  const TickType_t transmit_freq = MAX_WAIT_TIME(frequency_hz);
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    TransmitFrame_t frame = {0};
    frame.payload[0] = (uint8_t)ID_DATA_PACKET;

    memcpy(&frame.payload[1], &latest_data_packet, sizeof(latest_data_packet));

    frame.payload_len = (uint16_t)(1U + sizeof(DataPacket_t));
    if (frame.payload_len <= MAX_TRANSMIT_PAYLOAD_LEN) {
      (void)xQueueSend(transmit_queue, &frame, 0);
    }

    vTaskDelayUntil(&last_wake_time, transmit_freq);
  }
}
